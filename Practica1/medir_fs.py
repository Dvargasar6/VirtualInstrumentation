"""
Paso 3: Adquisicion + demodulacion + FFT con ejes correctos.

Mejoras respecto al codigo original:
  - fs medida empiricamente al arranque (no asumida).
  - Hilos separados para lectura serial, transmision PWM y plotting.
  - Filtro Butterworth disenado con la fs real.
  - FFT con ejes centrados en frecuencias esperadas (no autoescalados).
  - Plot refrescado a tasa fija (no en cada muestra) para no saturar la CPU.
  - Uso de set_data + relim/autoscale en vez de clear+plot en cada frame
    (mucho mas eficiente con matplotlib).
"""

import serial
import time
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt   # filtro IIR + filtrado sin desfase
from collections import deque

# ============================================================
# CONFIGURACION
# ============================================================
PUERTO   = '/dev/ttyUSB0'
BAUDRATE = 230400

# Parametros AM (deben coincidir con el transmisor)
AMPLITUD_PORTADORA  = 60
AMPLITUD_MODULADORA = 60
FRECUENCIA_PORTADORA  = 60   # fc [Hz]
FRECUENCIA_MODULADORA = 10   # fm [Hz]
OFFSET                = 120  # nivel DC en escala DAC (0-255)

# Buffer circular. 1024 muestras a ~385 Hz = ~2.66 s de ventana.
# Resolucion FFT = fs/N = 385/1024 = 0.376 Hz/bin (sobra para separar 70/80/90 Hz).
BUFFER_SIZE = 1024
buffer_adc  = deque(maxlen=BUFFER_SIZE)
buffer_lock = threading.Lock()

# Event para senalizar parada a los hilos
running   = threading.Event()
fs_medida = 0.0                # se rellena tras calibracion


# ============================================================
# CALIBRACION
# ============================================================
def medir_fs(conn, duracion=3.0):
    """Mide fs real del ADC sin TX activo."""
    print(f"Calibrando fs durante {duracion} s...")
    conn.reset_input_buffer()
    conn.write(b"START_ADC\n")
    time.sleep(0.2)
    conn.reset_input_buffer()              # descarta "ADC_STREAMING_ON"

    n = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < duracion:
        linea = conn.readline().decode(errors='ignore').strip()
        if not linea:
            continue
        try:
            float(linea)
            n += 1
        except ValueError:
            pass
    t_real = time.perf_counter() - t0

    conn.write(b"STOP_ADC\n")
    time.sleep(0.2)
    conn.reset_input_buffer()

    fs = n / t_real
    print(f"fs real: {fs:.2f} Hz ({n} muestras en {t_real:.2f} s)")
    return fs


# ============================================================
# PROCESAMIENTO DSP
# ============================================================
def demodular_am(signal, fs, fc_filtro):
    """
    Demodulador de envolvente:
      1. Rectificar (valor absoluto): la portadora rectificada deja una
         componente DC proporcional a la envolvente + armonicos de 2*fc.
      2. Filtro paso-bajo: elimina los armonicos de la portadora y deja
         solo la moduladora.
      3. Quitar DC residual (resta de la media).

    Parametros:
        signal    : array de muestras del ADC ya con DC removido
        fs        : tasa de muestreo real [Hz]
        fc_filtro : frecuencia de corte del LP [Hz], tipicamente 2*fm
    """
    # 1. Rectificacion de media onda inversa (valor absoluto)
    rectificada = np.abs(signal)

    # 2. Diseno del filtro Butterworth orden 4
    # Wn debe estar normalizado a Nyquist: Wn = fc / (fs/2)
    nyq = fs / 2.0
    Wn  = fc_filtro / nyq
    # Verificacion defensiva: el filtro digital requiere 0 < Wn < 1
    if not (0 < Wn < 1):
        raise ValueError(f"fc_filtro={fc_filtro} Hz invalido para fs={fs} Hz")
    # butter devuelve coeficientes b, a del filtro IIR
    b, a = butter(N=4, Wn=Wn, btype='low')
    # filtfilt aplica el filtro dos veces (adelante/atras) -> fase cero
    envolvente = filtfilt(b, a, rectificada)

    # 3. Quitar DC residual (la moduladora pura tiene media cero)
    envolvente = envolvente - envolvente.mean()

    return envolvente


def calcular_fft(signal, fs):
    """
    FFT de una sola cara con ventana Hanning.
    Devuelve (frecuencias, magnitudes) normalizadas correctamente.
    """
    N = len(signal)
    # Ventana Hanning: reduce leakage espectral por discontinuidad en los bordes
    ventana = np.hanning(N)
    # rfft: solo bins positivos (la senal es real, los negativos son conjugados)
    yf = np.fft.rfft(signal * ventana)
    # rfftfreq: frecuencias correspondientes a cada bin de rfft
    xf = np.fft.rfftfreq(N, d=1.0/fs)
    # Normalizacion: factor 2 por usar solo media banda + correccion de ventana
    # La ganancia coherente de Hanning es 0.5, asi que multiplicamos por 2/0.5 = 4
    # pero ademas dividimos por N por la propia FFT. Resultado: amplitud en
    # unidades de la senal original para senales sinusoidales.
    magnitud = (2.0 * np.abs(yf)) / (N * 0.5)
    return xf, magnitud


def detectar_pico(xf, mag, f_objetivo, busqueda=5.0):
    """
    Encuentra el pico real cerca de una frecuencia objetivo y devuelve
    su frecuencia con precision sub-bin mediante interpolacion parabolica.

    Por que interpolacion: la FFT tiene resolucion fs/N (ej. 0.376 Hz para
    N=1024, fs=385). Si el pico real esta entre dos bins, leer solo el
    indice del maximo da error de hasta +/- 0.188 Hz. Ajustar una parabola
    a los tres bins (max-1, max, max+1) recupera la frecuencia exacta con
    error << bin.

    Parametros:
        xf, mag    : arrays de la FFT (frecuencia, magnitud)
        f_objetivo : frecuencia teorica esperada [Hz]
        busqueda   : ancho de la ventana de busqueda alrededor de f_objetivo [Hz]

    Devuelve:
        (f_pico, mag_pico) o (None, None) si no hay pico claro en la ventana
    """
    # Mascara de la ventana de busqueda
    mask = (xf >= f_objetivo - busqueda) & (xf <= f_objetivo + busqueda)
    if not mask.any():
        return None, None
    indices = np.where(mask)[0]
    # Indice del bin con mayor magnitud dentro de la ventana
    i_local = np.argmax(mag[indices])
    k = indices[i_local]

    # Interpolacion parabolica (formula de Quinn / Jain).
    # Necesita los vecinos k-1 y k+1; si k esta en el borde, devolvemos
    # directamente el bin sin interpolar.
    if k <= 0 or k >= len(mag) - 1:
        return xf[k], mag[k]

    y0, y1, y2 = mag[k-1], mag[k], mag[k+1]
    denom = (y0 - 2*y1 + y2)
    if denom == 0:
        # Parabola degenerada (todos iguales); devolvemos el bin sin correccion
        return xf[k], y1
    # Desplazamiento sub-bin: en [-0.5, 0.5] respecto al bin central
    delta = 0.5 * (y0 - y2) / denom
    # Conversion a Hz: el espaciado entre bins es xf[1]-xf[0]
    df = xf[1] - xf[0]
    f_pico   = xf[k] + delta * df
    # Magnitud interpolada (formula del vertice de la parabola)
    mag_pico = y1 - 0.25 * (y0 - y2) * delta
    return f_pico, mag_pico


# ============================================================
# HILOS
# ============================================================
def hilo_lector(conn):
    """Productor: lee del serial y mete en el buffer."""
    while running.is_set():
        try:
            linea = conn.readline().decode(errors='ignore').strip()
            if not linea:
                continue
            valor = float(linea)
            with buffer_lock:
                buffer_adc.append(valor)
        except ValueError:
            pass
        except Exception as e:
            print(f"[lector] {e}")
            break


def hilo_transmisor(conn):
    """
    Genera senal AM y la envia al DAC del ESP32.

    IMPORTANTE: el tiempo 't' se calcula con perf_counter (reloj real
    monotonico), no con un contador de muestras. Esto garantiza que las
    frecuencias de la senal sean EXACTAS aunque el envio sea irregular
    por latencia del UART, GIL, scheduler de Linux, etc.

    Si usaramos t = n/fs como antes, cualquier dilatacion temporal del
    envio se traduciria directamente en una compresion de la frecuencia
    real (lo que pasaba con fm=10 Hz aparecciendo como 3.4 Hz).
    """
    Ac, Am = AMPLITUD_PORTADORA, AMPLITUD_MODULADORA
    fc, fm = FRECUENCIA_PORTADORA, FRECUENCIA_MODULADORA

    # Periodo objetivo entre envios. No determina la frecuencia generada
    # (eso lo hace perf_counter), pero si limita la tasa de comandos al
    # ESP32 para no saturar el UART y dejar ancho de banda al ADC.
    # A 230400 baud, ~12 bytes/cmd = ~520 us minimo. Pedimos 2 ms = 500 Hz.
    PERIODO_TX = 0.002                     # segundos

    t0 = time.perf_counter()               # tiempo de referencia inicial

    while running.is_set():
        # Tiempo real desde el inicio. Esto es lo que define la frecuencia
        # de la senal generada, independiente de la regularidad del loop.
        t = time.perf_counter() - t0

        moduladora = np.sin(2 * np.pi * fm * t)
        portadora  = np.sin(2 * np.pi * fc * t)
        # AM convencional: s(t) = Vdc + Ac(1 + m*moduladora) * portadora
        s = OFFSET + Ac * (1 + (Am / Ac) * moduladora) * portadora
        valor = int(np.clip(s, 0, 255))
        try:
            conn.write(f"SET_PWM:{valor}\n".encode())
        except Exception as e:
            print(f"[tx] {e}")
            break

        # Sleep para no saturar UART. La frecuencia generada sigue siendo
        # exacta porque 't' se calcula con perf_counter en cada iteracion.
        time.sleep(PERIODO_TX)


# ============================================================
# MAIN
# ============================================================
def main():
    global fs_medida

    conn = serial.Serial(PUERTO, BAUDRATE, timeout=0.05)
    time.sleep(2)                          # reset auto del ESP32

    # Handshake
    conn.reset_input_buffer()
    conn.write(b"PING\n")
    time.sleep(0.3)
    if conn.readline().decode(errors='ignore').strip() != "PONG":
        print("ESP32 no respondio. Abortando.")
        conn.close()
        return
    print("ESP32 conectado.")

    # Calibracion
    fs_medida = medir_fs(conn, duracion=3.0)
    if fs_medida < 2 * FRECUENCIA_PORTADORA:
        print(f"fs={fs_medida:.1f} Hz insuficiente (Nyquist={2*FRECUENCIA_PORTADORA} Hz).")
        conn.close()
        return

    # Arranque de hilos
    running.set()
    conn.write(b"START_ADC\n")
    time.sleep(0.1)
    conn.reset_input_buffer()
    threading.Thread(target=hilo_lector,     args=(conn,), daemon=True).start()
    threading.Thread(target=hilo_transmisor, args=(conn,), daemon=True).start()

    # ----- Configuracion de los 4 subplots -----
    plt.ion()
    fig, axs = plt.subplots(4, 1, figsize=(12, 11))
    fig.tight_layout(pad=2.5)

    # Lineas principales: las creamos una vez y solo actualizamos sus datos.
    # Esto es mucho mas eficiente que axs[i].clear() + axs[i].plot() por frame.
    linea_am,        = axs[0].plot([], [], lw=0.8, color='steelblue')
    linea_fft_am,    = axs[1].plot([], [], lw=1.0, color='coral')
    linea_demod,     = axs[2].plot([], [], lw=1.0, color='mediumseagreen')
    linea_fft_demod, = axs[3].plot([], [], lw=1.0, color='mediumpurple')

    # Frecuencias teoricas (basadas en las constantes globales)
    fc = FRECUENCIA_PORTADORA
    fm = FRECUENCIA_MODULADORA

    # Plot 0: senal AM en tiempo
    axs[0].set_title(f"Senal AM cruda (fs = {fs_medida:.1f} Hz)")
    axs[0].set_xlabel("Tiempo [s]"); axs[0].set_ylabel("Voltaje [V]")
    axs[0].grid(True, alpha=0.4)

    # Plot 1: FFT AM. Eje X centrado en fc con margen para ver bandas laterales.
    margen_am = max(30, 3 * fm)
    axs[1].set_xlabel("Frecuencia [Hz]"); axs[1].set_ylabel("Magnitud")
    axs[1].set_xlim(fc - margen_am, fc + margen_am)
    axs[1].grid(True, alpha=0.4)

    # Marcadores verticales TEORICOS (rojos, punteados). Los guardamos como
    # objetos para poder actualizarlos si las constantes cambian dinamicamente.
    # Ahora se generan en bucle, no hardcodeados, asi que cambiar fc o fm
    # automaticamente reubica las lineas.
    f_teoricas_am = [fc - fm, fc, fc + fm]
    etiquetas_am  = [f"fc-fm\n{fc-fm} Hz", f"fc\n{fc} Hz", f"fc+fm\n{fc+fm} Hz"]
    vlines_teoricas_am = []
    textos_teoricos_am = []
    for f, etiq in zip(f_teoricas_am, etiquetas_am):
        vl = axs[1].axvline(f, color='red', linestyle='--', alpha=0.5, lw=0.8)
        txt = axs[1].text(f, 0, etiq, color='red', fontsize=7,
                          ha='center', va='bottom', alpha=0.7)
        vlines_teoricas_am.append(vl)
        textos_teoricos_am.append(txt)

    # Marcadores de picos REALES detectados (verdes, solidos). Inicialmente
    # invisibles; los movemos cuando detectamos los picos en cada frame.
    vlines_reales_am = []
    textos_reales_am = []
    for _ in f_teoricas_am:
        vl = axs[1].axvline(0, color='green', linestyle='-', alpha=0.6, lw=0.8,
                            visible=False)
        txt = axs[1].text(0, 0, "", color='green', fontsize=7,
                          ha='center', va='top', visible=False)
        vlines_reales_am.append(vl)
        textos_reales_am.append(txt)

    # Titulo dinamico que actualizaremos con los errores
    axs[1].set_title("FFT AM | esperando muestras...")

    # Plot 2: senal demodulada en tiempo
    axs[2].set_title(f"Senal demodulada (filtro LP corte = {2*fm} Hz)")
    axs[2].set_xlabel("Tiempo [s]"); axs[2].set_ylabel("Amplitud")
    axs[2].grid(True, alpha=0.4)

    # Plot 3: FFT demodulada. Eje X de 0 a 3*fm.
    margen_demod = 3 * fm
    axs[3].set_xlabel("Frecuencia [Hz]"); axs[3].set_ylabel("Magnitud")
    axs[3].set_xlim(0, margen_demod)
    axs[3].grid(True, alpha=0.4)

    # Marcador teorico de fm
    vline_teorica_demod = axs[3].axvline(fm, color='purple', linestyle='--',
                                          alpha=0.6, lw=0.9)
    texto_teorico_demod = axs[3].text(fm, 0, f"fm teorico\n{fm} Hz",
                                       color='purple', fontsize=7,
                                       ha='center', va='bottom', alpha=0.7)
    # Marcador real
    vline_real_demod = axs[3].axvline(0, color='green', linestyle='-',
                                       alpha=0.6, lw=0.9, visible=False)
    texto_real_demod = axs[3].text(0, 0, "", color='green', fontsize=7,
                                    ha='center', va='top', visible=False)
    axs[3].set_title("FFT demodulada | esperando muestras...")

    # ----- Loop principal de actualizacion -----
    INTERVALO_PLOT = 0.1                   # 10 Hz de refresco
    try:
        while True:
            with buffer_lock:
                muestras = np.array(buffer_adc)

            # Necesitamos un minimo razonable de muestras para FFT util
            # (al menos 2 ciclos de fm = 2*fs/fm muestras)
            min_muestras = int(2 * fs_medida / fm)
            if len(muestras) < min_muestras:
                plt.pause(INTERVALO_PLOT)
                continue

            # ----- Procesamiento -----
            # Quitar DC antes de FFT y demodulacion (el bin 0 dominaria todo)
            muestras_ac = muestras - muestras.mean()
            t_axis      = np.arange(len(muestras_ac)) / fs_medida

            # Demodulacion con filtro disenado con la fs REAL
            demodulada = demodular_am(muestras_ac, fs_medida, fc_filtro=2*fm)

            # FFTs
            xf_am,    mag_am    = calcular_fft(muestras_ac, fs_medida)
            xf_demod, mag_demod = calcular_fft(demodulada,  fs_medida)

            # ----- Actualizacion de plots (set_data, sin clear) -----
            linea_am.set_data(t_axis, muestras)
            axs[0].set_xlim(0, t_axis[-1])
            axs[0].set_ylim(muestras.min() - 0.05, muestras.max() + 0.05)

            linea_fft_am.set_data(xf_am, mag_am)
            # Ylim dinamico sobre la ventana visible (ignorar picos fuera)
            mask_am = (xf_am >= fc - margen_am) & (xf_am <= fc + margen_am)
            if mask_am.any():
                ymax_am = mag_am[mask_am].max() * 1.20
                axs[1].set_ylim(0, ymax_am if ymax_am > 0 else 1)

            # Deteccion de los tres picos del AM y calculo de errores
            # Ventana de busqueda = fm/2 para evitar que los picos vecinos
            # se interfieran (las bandas laterales estan separadas fm Hz).
            errores_am = []
            for i, f_teo in enumerate(f_teoricas_am):
                f_real, mag_real = detectar_pico(xf_am, mag_am,
                                                  f_teo, busqueda=fm/2)
                if f_real is not None:
                    # Reposicionar marcador y etiqueta del pico real.
                    # Usamos set_xdata con secuencia para axvline (matplotlib lo exige).
                    vlines_reales_am[i].set_xdata([f_real, f_real])
                    vlines_reales_am[i].set_visible(True)
                    textos_reales_am[i].set_position((f_real, ymax_am * 0.95))
                    textos_reales_am[i].set_text(f"{f_real:.2f} Hz")
                    textos_reales_am[i].set_visible(True)
                    # Reposicionar etiqueta teorica al tope del plot
                    textos_teoricos_am[i].set_position((f_teo, ymax_am * 0.05))
                    # Error porcentual relativo al valor teorico
                    err_pct = 100.0 * (f_real - f_teo) / f_teo
                    errores_am.append(err_pct)
                else:
                    vlines_reales_am[i].set_visible(False)
                    textos_reales_am[i].set_visible(False)
                    errores_am.append(None)

            # Titulo con los errores. Si algun pico no se detecto, mostramos "--".
            err_str = " | ".join(
                f"{'--' if e is None else f'{e:+.2f}%'}"
                for e in errores_am
            )
            axs[1].set_title(
                f"FFT AM | picos teoricos: {fc-fm}/{fc}/{fc+fm} Hz "
                f"| errores: {err_str}"
            )

            linea_demod.set_data(t_axis, demodulada)
            axs[2].set_xlim(0, t_axis[-1])
            axs[2].set_ylim(demodulada.min() * 1.1 - 0.01,
                            demodulada.max() * 1.1 + 0.01)

            linea_fft_demod.set_data(xf_demod, mag_demod)
            mask_demod = (xf_demod >= 0) & (xf_demod <= margen_demod)
            if mask_demod.any():
                ymax_demod = mag_demod[mask_demod].max() * 1.20
                axs[3].set_ylim(0, ymax_demod if ymax_demod > 0 else 1)

            # Deteccion del pico de fm. Ventana = fm/2 para no confundir
            # con armonicos o residuos cerca de cero.
            f_real_demod, _ = detectar_pico(xf_demod, mag_demod,
                                             fm, busqueda=fm/2)
            if f_real_demod is not None:
                vline_real_demod.set_xdata([f_real_demod, f_real_demod])
                vline_real_demod.set_visible(True)
                texto_real_demod.set_position((f_real_demod, ymax_demod * 0.95))
                texto_real_demod.set_text(f"{f_real_demod:.2f} Hz")
                texto_real_demod.set_visible(True)
                texto_teorico_demod.set_position((fm, ymax_demod * 0.05))
                err_demod = 100.0 * (f_real_demod - fm) / fm
                axs[3].set_title(
                    f"FFT demodulada | teorico: {fm} Hz "
                    f"| real: {f_real_demod:.2f} Hz "
                    f"| error: {err_demod:+.2f}%"
                )
            else:
                vline_real_demod.set_visible(False)
                texto_real_demod.set_visible(False)
                axs[3].set_title(f"FFT demodulada | teorico {fm} Hz | pico no detectado")

            fig.canvas.draw_idle()
            plt.pause(INTERVALO_PLOT)

    except KeyboardInterrupt:
        print("\nDeteniendo...")
    finally:
        running.clear()
        time.sleep(0.3)
        try:
            conn.write(b"STOP_ADC\n")
            conn.write(b"DAC_OFF\n")
            conn.close()
        except Exception:
            pass
        print("Puerto cerrado.")


if __name__ == "__main__":
    main()