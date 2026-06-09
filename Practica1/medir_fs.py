import serial
import time
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt   # filtro IIR + filtrado sin desfase
from collections import deque
import tkinter as tk                        # GUI para sliders de control
from tkinter import ttk                     # widgets con tema mejorado

# ============================================================
# CONFIGURACION
# ============================================================
PUERTO   = '/dev/ttyUSB0'
BAUDRATE = 230400

# Parametros AM INICIALES. A diferencia de antes, ahora la generacion se hace
# en el ESP32 y estos valores se envian por SET_AM. Los sliders del Tkinter
# permiten cambiarlos en vivo durante la ejecucion.
AM_INICIAL = {
    'Ac':     60.0,   # amplitud portadora (escala DAC 0-255)
    'Am':     60.0,   # amplitud moduladora
    'fc':     80.0,   # frecuencia portadora [Hz]
    'fm':     10.0,   # frecuencia moduladora [Hz]
    'offset': 120.0,  # nivel DC (escala DAC 0-255)
}

# Rangos validos de cada parametro para los sliders. Los limites superiores
# de Ac+offset deben respetar 0 <= valor_DAC <= 255 (8 bits del DAC del ESP32).
AM_RANGOS = {
    'Ac':     (0.0,   100.0),
    'Am':     (0.0,   100.0),
    'fc':     (1.0,   500.0),   # tope = ~fs_adc/4 para evitar aliasing severo
    'fm':     (0.1,    50.0),   # tope = fc/2 para mantener AM convencional
    'offset': (0.0,   255.0),
}

# Parametros AM "vivos": objeto compartido entre el hilo GUI (escritor) y el
# hilo controlador (lector). Protegido por su propio lock.
am_params      = dict(AM_INICIAL)
am_lock        = threading.Lock()
am_dirty       = threading.Event()  # se setea cuando un slider cambia
# fc y fm se necesitan tambien en el hilo de plotting para las lineas teoricas.
# El plotting los lee directamente de am_params bajo am_lock.

# Constantes del ADC del ESP32. El firmware envia el valor crudo (0-4095)
# y aqui hacemos la conversion a voltios. Esto es mas eficiente que enviar
# floats por UART (menos bytes, mas muestras por segundo).
ADC_MAX     = 4095            # 12 bits del ADC del ESP32
ADC_VREF    = 3.3             # tension de referencia en voltios
ADC_FACTOR  = ADC_VREF / ADC_MAX   # se precalcula para no dividir por muestra

# Buffer circular. Tamano elegido para mantener ~2-3 s de ventana temporal
# independientemente de la fs. A fs~1500 Hz, 4096 muestras = ~2.7 s, suficiente
# para ver ~27 ciclos de la moduladora de 10 Hz. La resolucion FFT correspondiente
# es fs/N ~= 0.37 Hz/bin, suficiente para separar fc, fc-fm y fc+fm.
BUFFER_SIZE = 4096
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
            # El firmware ahora envia entero ADC (0-4095), no float.
            # int() rechaza strings con punto decimal: util para detectar
            # si hay un firmware antiguo conectado por error.
            int(linea)
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
            # El firmware envia entero ADC (0-4095). Convertimos a voltios aqui
            # multiplicando por la constante precalculada ADC_FACTOR = 3.3/4095.
            adc_raw = int(linea)
            voltaje = adc_raw * ADC_FACTOR
            with buffer_lock:
                buffer_adc.append(voltaje)
        except ValueError:
            pass
        except Exception as e:
            print(f"[lector] {e}")
            break


def hilo_controlador_am(conn):
    """
    Envia comandos SET_AM al ESP32 cuando los sliders cambian. Mucho mas
    eficiente que el transmisor anterior, que enviaba un comando SET_PWM por
    cada muestra. Ahora solo se envia un SET_AM por cambio de slider.

    Funcionamiento:
      - Espera a que la GUI senalice un cambio via am_dirty.set()
      - Lee am_params bajo lock
      - Construye el comando SET_AM:Ac,Am,fc,fm,offset
      - Lo envia al ESP32
      - Limpia el flag y vuelve a esperar
    """
    # Antes de iniciar, enviar los parametros iniciales y arrancar la AM
    with am_lock:
        cmd = ("SET_AM:"
               f"{am_params['Ac']:.2f},"
               f"{am_params['Am']:.2f},"
               f"{am_params['fc']:.2f},"
               f"{am_params['fm']:.2f},"
               f"{am_params['offset']:.2f}\n")
    try:
        conn.write(cmd.encode())
        time.sleep(0.05)                   # margen para que el ESP32 procese
        conn.write(b"START_AM\n")
    except Exception as e:
        print(f"[ctrl_am] error inicial: {e}")
        return

    while running.is_set():
        # wait con timeout permite revisar running periodicamente y salir limpio
        if not am_dirty.wait(timeout=0.2):
            continue
        am_dirty.clear()

        # Snapshot atomico de los parametros actuales
        with am_lock:
            cmd = ("SET_AM:"
                   f"{am_params['Ac']:.2f},"
                   f"{am_params['Am']:.2f},"
                   f"{am_params['fc']:.2f},"
                   f"{am_params['fm']:.2f},"
                   f"{am_params['offset']:.2f}\n")
        try:
            conn.write(cmd.encode())
        except Exception as e:
            print(f"[ctrl_am] {e}")
            break


# ============================================================
# MAIN
# ============================================================
def hilo_gui_tkinter():
    """
    Crea y ejecuta la ventana Tkinter con los sliders de control AM.
    Esta funcion se ejecuta en su propio hilo porque mainloop() bloquea.

    Mecanica:
      - Cada slider modifica una entrada de am_params bajo am_lock.
      - Tras cualquier cambio se hace am_dirty.set() para que el hilo
        controlador envie el comando SET_AM al ESP32.
      - Hay un debounce de 100 ms para no inundar el UART con cambios
        rapidos de slider (el evento de Scale se dispara en cada pixel
        de movimiento del cursor).
    """
    root = tk.Tk()
    root.title("Control AM - ESP32")
    root.geometry("450x420")
    # Protocolo de cierre: si el usuario cierra la ventana, se senala parada
    # global. Asi el resto del programa termina limpiamente.
    root.protocol("WM_DELETE_WINDOW", lambda: (running.clear(), root.destroy()))

    # Frame contenedor con padding
    frame = ttk.Frame(root, padding=15)
    frame.pack(fill=tk.BOTH, expand=True)

    # Titulo
    titulo = ttk.Label(frame, text="Parametros de la senal AM",
                       font=('Sans', 12, 'bold'))
    titulo.pack(pady=(0, 10))

    # Variable de debounce. Guarda el id del 'after' programado para
    # cancelarlo si llega otro cambio antes de que dispare.
    debounce_id = [None]
    DEBOUNCE_MS = 100

    def disparar_envio():
        """Marca los parametros como sucios para que el hilo controlador envie."""
        am_dirty.set()
        debounce_id[0] = None

    def crear_slider(parent, nombre, etiqueta, unidad):
        """
        Crea un slider con etiqueta, valor numerico y rango. Devuelve el
        widget Scale por si hace falta acceder a el despues.
        """
        # Sub-frame para esta fila
        fila = ttk.Frame(parent)
        fila.pack(fill=tk.X, pady=4)

        # Etiqueta a la izquierda con nombre fijo
        lbl_nombre = ttk.Label(fila, text=etiqueta, width=22, anchor='w')
        lbl_nombre.pack(side=tk.LEFT)

        # Etiqueta del valor actual (se actualiza en tiempo real)
        valor_var = tk.StringVar(value=f"{AM_INICIAL[nombre]:.2f} {unidad}")
        lbl_valor = ttk.Label(fila, textvariable=valor_var, width=12, anchor='e',
                              font=('Mono', 10))
        lbl_valor.pack(side=tk.RIGHT)

        # Callback que se ejecuta en CADA movimiento del slider
        def on_change(val_str):
            v = float(val_str)
            valor_var.set(f"{v:.2f} {unidad}")
            # Actualizar el dict compartido bajo lock
            with am_lock:
                am_params[nombre] = v
            # Debounce: cancelar envio anterior si lo hay, programar uno nuevo
            if debounce_id[0] is not None:
                root.after_cancel(debounce_id[0])
            debounce_id[0] = root.after(DEBOUNCE_MS, disparar_envio)

        rango_min, rango_max = AM_RANGOS[nombre]
        scale = tk.Scale(parent, from_=rango_min, to=rango_max,
                         orient=tk.HORIZONTAL, resolution=0.1,
                         command=on_change, length=400, sliderlength=20)
        scale.set(AM_INICIAL[nombre])
        scale.pack(fill=tk.X, pady=(0, 8))
        return scale

    # Crear los 5 sliders
    crear_slider(frame, 'Ac',     "Amplitud portadora (Ac):",   "")
    crear_slider(frame, 'Am',     "Amplitud moduladora (Am):",  "")
    crear_slider(frame, 'fc',     "Frecuencia portadora (fc):", "Hz")
    crear_slider(frame, 'fm',     "Frecuencia moduladora (fm):","Hz")
    crear_slider(frame, 'offset', "Offset DC:",                 "")

    # Boton para resetear a los valores iniciales
    def reset():
        # Recargar valores iniciales y forzar actualizacion de la GUI.
        # La forma mas robusta es destruir y recrear la ventana, pero como
        # eso es disruptivo, usamos un enfoque mas suave: actualizar cada
        # slider via su widget. Recorremos los hijos del frame y buscamos
        # los Scale.
        # Aqui simplificamos: actualizamos am_params y forzamos un SET_AM.
        # El usuario vera los sliders desactualizados visualmente, pero el
        # ESP32 recibira los valores correctos. Para evitar inconsistencia
        # es preferible cerrar y reabrir la ventana.
        with am_lock:
            am_params.update(AM_INICIAL)
        am_dirty.set()

    btn_reset = ttk.Button(frame, text="Resetear a valores iniciales",
                            command=reset)
    btn_reset.pack(pady=(10, 0))

    # Iniciar el loop de eventos de Tkinter. Esta llamada bloquea hasta
    # que la ventana se cierra.
    root.mainloop()


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
    # Validar Nyquist contra la fc inicial. Si el usuario sube fc despues con
    # el slider, el rango del slider ya respeta el limite (configurado en AM_RANGOS).
    fc_inicial = AM_INICIAL['fc']
    if fs_medida < 2 * fc_inicial:
        print(f"fs={fs_medida:.1f} Hz insuficiente (Nyquist={2*fc_inicial} Hz).")
        conn.close()
        return

    # Arranque de hilos
    running.set()
    conn.write(b"START_ADC\n")
    time.sleep(0.1)
    conn.reset_input_buffer()
    threading.Thread(target=hilo_lector,         args=(conn,), daemon=True).start()
    threading.Thread(target=hilo_controlador_am, args=(conn,), daemon=True).start()
    # GUI Tkinter en hilo aparte. mainloop() bloquea, asi que va en su propio
    # hilo. matplotlib se queda en el main thread (requisito de su backend).
    threading.Thread(target=hilo_gui_tkinter, daemon=True).start()

    # ----- Configuracion de los 4 subplots -----
    plt.ion()
    fig, axs = plt.subplots(4, 1, figsize=(12, 11))
    fig.tight_layout(pad=2.5)

    # Lineas principales: las creamos una vez y solo actualizamos sus datos.
    # Esto es mucho mas eficiente que axs[i].clear() + axs[i].plot() por frame.
    # Plot 0: senal cruda + envolvente superpuesta en rojo translucido
    linea_am,        = axs[0].plot([], [], lw=1.0, color='steelblue',
                                    alpha=0.7, label='Senal AM')
    linea_envolvente,= axs[0].plot([], [], lw=1.8, color='crimson',
                                    alpha=0.9, label='Envolvente')
    axs[0].legend(loc='upper right', fontsize=8, framealpha=0.9)
    linea_fft_am,    = axs[1].plot([], [], lw=1.0, color='coral')
    linea_demod,     = axs[2].plot([], [], lw=1.2, color='mediumseagreen')
    linea_fft_demod, = axs[3].plot([], [], lw=1.0, color='mediumpurple')

    # Frecuencias teoricas iniciales. Los sliders pueden cambiarlas en vivo,
    # asi que cada iteracion del loop principal las relee de am_params.
    # Aqui solo se usan para el setup inicial de los plots.
    with am_lock:
        fc = am_params['fc']
        fm = am_params['fm']

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

    # Variables para fijar los Y de los 4 plots tras una calibracion inicial.
    # Estrategia: durante los primeros TIEMPO_CALIB_Y segundos despues de tener
    # buffer lleno, acumulamos estadisticas. Tras eso, fijamos los limites y
    # no los volvemos a cambiar. Esto permite ver visualmente cuando la senal
    # desaparece (al bloquear el acople optico) porque los limites quedan
    # anclados al rango "normal" de operacion.
    TIEMPO_CALIB_Y = 2.0                   # segundos para calibrar Y
    SEMIRANGO_AM   = 1.0                   # +/- 1.0 V alrededor de la media
    FACTOR_MARGEN  = 1.5                   # tope = pico_observado * este factor
    t_inicio_calib = None                  # se setea cuando llega el primer frame
    y_calibrado    = False                 # flag: ya fijamos los limites
    y_am_min       = None                  # limites finales del plot AM (tiempo)
    y_am_max       = None
    y_demod_lim    = None                  # limite +/- para plot demodulada (tiempo)
    y_fft_am_max   = None                  # tope del eje Y de la FFT AM
    y_fft_demod_max= None                  # tope del eje Y de la FFT demodulada

    # Valores anteriores de fc/fm para detectar cambios entre frames.
    # Si cambian, reconfiguramos los ejes X y las lineas teoricas.
    fc_prev = fc
    fm_prev = fm

    try:
        while True:
            # Releer fc y fm cada frame. Los sliders pueden haberlos cambiado.
            with am_lock:
                fc = am_params['fc']
                fm = am_params['fm']

            # Si fc o fm cambiaron, reconfigurar plots dependientes.
            if fc != fc_prev or fm != fm_prev:
                # Reposicionar lineas teoricas y etiquetas del plot FFT AM
                f_teoricas_am_actual = [fc - fm, fc, fc + fm]
                etiquetas_actual = [f"fc-fm\n{fc-fm:.1f} Hz",
                                     f"fc\n{fc:.1f} Hz",
                                     f"fc+fm\n{fc+fm:.1f} Hz"]
                for i, (f, etiq) in enumerate(zip(f_teoricas_am_actual, etiquetas_actual)):
                    vlines_teoricas_am[i].set_xdata([f, f])
                    textos_teoricos_am[i].set_position((f, 0))
                    textos_teoricos_am[i].set_text(etiq)

                # Reposicionar linea teorica del plot FFT demodulada
                vline_teorica_demod.set_xdata([fm, fm])
                texto_teorico_demod.set_position((fm, 0))
                texto_teorico_demod.set_text(f"fm teorico\n{fm:.1f} Hz")

                # Actualizar limites de los ejes X
                margen_am    = max(30, 3 * fm)
                margen_demod = 3 * fm
                axs[1].set_xlim(fc - margen_am, fc + margen_am)
                axs[3].set_xlim(0, margen_demod)

                # Actualizar lista de frecuencias teoricas para deteccion de picos
                f_teoricas_am = f_teoricas_am_actual

                fc_prev = fc
                fm_prev = fm

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
            # Plot 0: senal AM cruda + envolvente. La envolvente se obtiene
            # de la misma demodulacion ya calculada, pero RE-AGREGANDO la media
            # original que le quitamos a las muestras_ac. Asi la envolvente
            # queda visualmente centrada sobre la senal cruda en V reales.
            media_actual = muestras.mean()
            envolvente_visual = demodulada + media_actual
            linea_am.set_data(t_axis, muestras)
            linea_envolvente.set_data(t_axis, envolvente_visual)
            axs[0].set_xlim(0, t_axis[-1])

            # Calibracion de Y (solo durante los primeros segundos).
            # Los 4 plots se fijan al mismo tiempo: tiempo AM, tiempo demodulada,
            # FFT AM y FFT demodulada. Asi al bloquear el acople optico se ve
            # de forma consistente como las cuatro graficas pierden la senal.
            if not y_calibrado:
                if t_inicio_calib is None:
                    t_inicio_calib = time.perf_counter()
                elif time.perf_counter() - t_inicio_calib >= TIEMPO_CALIB_Y:
                    # ----- Calibracion completada: fijar los 4 ejes Y -----

                    # Plot 0 (AM tiempo): +/- SEMIRANGO_AM volts alrededor de la media
                    y_am_min = media_actual - SEMIRANGO_AM
                    y_am_max = media_actual + SEMIRANGO_AM
                    axs[0].set_ylim(y_am_min, y_am_max)

                    # Plot 2 (demodulada tiempo): +/- (pico * factor) alrededor de 0
                    # Si la senal desaparece, el ruido residual quedara como
                    # una linea casi plana en el centro del eje
                    pico_demod_t = np.max(np.abs(demodulada))
                    y_demod_lim  = max(pico_demod_t * FACTOR_MARGEN, 0.1)
                    axs[2].set_ylim(-y_demod_lim, y_demod_lim)

                    # Plot 1 (FFT AM): tope = pico maximo en la ventana visible
                    # multiplicado por el factor de margen. La mascara restringe
                    # la busqueda al rango fc +/- margen_am para ignorar artefactos
                    # de baja frecuencia fuera de la zona de interes.
                    mask_calib_am = (xf_am >= fc - margen_am) & (xf_am <= fc + margen_am)
                    pico_fft_am   = mag_am[mask_calib_am].max() if mask_calib_am.any() else 0.0
                    y_fft_am_max  = max(pico_fft_am * FACTOR_MARGEN, 0.01)
                    axs[1].set_ylim(0, y_fft_am_max)

                    # Plot 3 (FFT demodulada): igual logica que la FFT AM
                    mask_calib_dm  = (xf_demod >= 0) & (xf_demod <= margen_demod)
                    pico_fft_demod = mag_demod[mask_calib_dm].max() if mask_calib_dm.any() else 0.0
                    y_fft_demod_max= max(pico_fft_demod * FACTOR_MARGEN, 0.01)
                    axs[3].set_ylim(0, y_fft_demod_max)

                    y_calibrado = True
                    print(f"Y AM tiempo  fijado en [{y_am_min:.3f}, {y_am_max:.3f}] V")
                    print(f"Y demod      fijado en +/- {y_demod_lim:.3f}")
                    print(f"Y FFT AM     fijado en [0, {y_fft_am_max:.4f}]")
                    print(f"Y FFT demod  fijado en [0, {y_fft_demod_max:.4f}]")
                else:
                    # Durante la calibracion: Y dinamico provisional en plot 0
                    axs[0].set_ylim(muestras.min() - 0.05, muestras.max() + 0.05)
            # Despues de calibrar: NO tocar mas el ylim del plot 0

            linea_fft_am.set_data(xf_am, mag_am)
            # Ylim dinamico solo durante calibracion; despues queda fijo.
            mask_am = (xf_am >= fc - margen_am) & (xf_am <= fc + margen_am)
            if not y_calibrado and mask_am.any():
                ymax_am = mag_am[mask_am].max() * 1.20
                axs[1].set_ylim(0, ymax_am if ymax_am > 0 else 1)
                ymax_label = ymax_am               # para posicionar etiquetas
            else:
                # Tras calibrar, las etiquetas se posicionan respecto al tope fijo
                ymax_label = y_fft_am_max if y_fft_am_max is not None else 1.0

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
                    textos_reales_am[i].set_position((f_real, ymax_label * 0.95))
                    textos_reales_am[i].set_text(f"{f_real:.2f} Hz")
                    textos_reales_am[i].set_visible(True)
                    # Reposicionar etiqueta teorica al tope del plot
                    textos_teoricos_am[i].set_position((f_teo, ymax_label * 0.05))
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
            # Y fijo despues de calibracion; dinamico durante la calibracion.
            if not y_calibrado:
                axs[2].set_ylim(demodulada.min() * 1.1 - 0.01,
                                demodulada.max() * 1.1 + 0.01)
            # Despues de calibrar: NO tocar mas el ylim del plot 2

            linea_fft_demod.set_data(xf_demod, mag_demod)
            mask_demod = (xf_demod >= 0) & (xf_demod <= margen_demod)
            # Ylim dinamico solo durante calibracion; despues queda fijo.
            if not y_calibrado and mask_demod.any():
                ymax_demod = mag_demod[mask_demod].max() * 1.20
                axs[3].set_ylim(0, ymax_demod if ymax_demod > 0 else 1)
                ymax_demod_label = ymax_demod
            else:
                ymax_demod_label = y_fft_demod_max if y_fft_demod_max is not None else 1.0

            # Deteccion del pico de fm. Ventana = fm/2 para no confundir
            # con armonicos o residuos cerca de cero.
            f_real_demod, _ = detectar_pico(xf_demod, mag_demod,
                                             fm, busqueda=fm/2)
            if f_real_demod is not None:
                vline_real_demod.set_xdata([f_real_demod, f_real_demod])
                vline_real_demod.set_visible(True)
                texto_real_demod.set_position((f_real_demod, ymax_demod_label * 0.95))
                texto_real_demod.set_text(f"{f_real_demod:.2f} Hz")
                texto_real_demod.set_visible(True)
                texto_teorico_demod.set_position((fm, ymax_demod_label * 0.05))
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
        am_dirty.set()                     # despertar al hilo controlador para que salga
        time.sleep(0.3)
        try:
            conn.write(b"STOP_AM\n")       # detener generacion AM en el ESP32
            conn.write(b"STOP_ADC\n")
            conn.write(b"DAC_OFF\n")
            conn.close()
        except Exception:
            pass
        print("Puerto cerrado.")


if __name__ == "__main__":
    main()