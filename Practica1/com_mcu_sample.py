import serial
import time
import numpy as np

PUERTO   = '/dev/ttyUSB0'   # En Windows sería 'COM3', 'COM4', etc.
BAUDRATE = 115200
TIMEOUT  = 2                # Segundos de espera para respuesta


def verificar_conexion(conexion: serial.Serial) -> bool:
    """
    Envía un PING al ESP32 y espera respuesta PONG.
    """
    print("Verificando conexión con el ESP32...")
    conexion.write("PING\n".encode('utf-8'))

    tiempo_inicio = time.time()
    while (time.time() - tiempo_inicio) < TIMEOUT:
        if conexion.in_waiting > 0:
            respuesta = conexion.readline().decode('utf-8').strip()
            if respuesta == "PONG":
                print("✓ Conexión verificada: ESP32 respondió correctamente.")
                return True
            else:
                print(f"Respuesta inesperada: '{respuesta}'")

    print("✗ Sin respuesta del ESP32. Verifica la conexión.")
    return False


def enviar_comando(conexion: serial.Serial, comando: str) -> str | None:
    """
    Envía un comando y espera respuesta.
    Retorna el texto de respuesta o None si hay timeout.
    """
    conexion.write(f"{comando}\n".encode('utf-8'))
    print(f"Comando enviado: '{comando}'")

    tiempo_inicio = time.time()
    while (time.time() - tiempo_inicio) < TIMEOUT:
        if conexion.in_waiting > 0:
            return conexion.readline().decode('utf-8').strip()

    print("Timeout: sin respuesta al comando.")
    return None
arduino-cli compile --fqbn esp32:esp32:esp32 esp32_firmware/ && \
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 esp32_firmware/

# ---------------------------------------------------------------------------
# PARÁMETROS DE LA ONDA AM
# ---------------------------------------------------------------------------
# Amplitud de la portadora (0–255).
AMPLITUD_PORTADORA  = 60    # Ac

# Amplitud de la moduladora (0–255). Índice m = Am/Ac, mantener m <= 1.
AMPLITUD_MODULADORA = 60    # Am

# Frecuencia de la portadora en Hz.
FRECUENCIA_PORTADORA  = 80   # fc [Hz]

# Frecuencia de la moduladora en Hz (debe ser < fc).
FRECUENCIA_MODULADORA = 20  # fm [Hz]

# Offset DC (0–255). Desplaza la señal hacia arriba para evitar valores
# negativos sin rectificar. Condición de no saturación:
#   OFFSET + Ac + Am <= 255
# Con los valores de arriba: 127 + 80 + 40 = 247 ✓ (margen de 8 unidades)
OFFSET = 120                # punto medio del DAC → señal centrada en ~1.65 V

# Tasa de muestreo: cuántas muestras por segundo se envían al DAC.
# Límite práctico con serial 115200 baud: ~450 sps.
SAMPLE_RATE = 50000           # fs [muestras/segundo]
# ---------------------------------------------------------------------------


def enviar_onda_am(
    conn     : serial.Serial,
    Ac       : float = AMPLITUD_PORTADORA,
    Am       : float = AMPLITUD_MODULADORA,
    fc       : float = FRECUENCIA_PORTADORA,
    fm       : float = FRECUENCIA_MODULADORA,
    offset   : int   = OFFSET,
    duracion : float = 60.0,
    fs       : int   = SAMPLE_RATE,
):
    """
    Envía una onda AM con offset DC como señal analógica al DAC (GPIO25).

    Fórmula:
        s(t) = offset + Ac * [1 + (Am/Ac) * sin(2*pi*fm*t)] * sin(2*pi*fc*t)

    El offset desplaza la señal hacia arriba para que nunca cruce el cero,
    preservando la forma sinusoidal completa (sin rectificacion).

    Condicion de no saturacion:
        offset + Ac + Am <= 255   (no satura el techo del DAC)
        offset - Ac - Am >= 0    (no satura el piso del DAC)

    Parámetros:
        Ac       : amplitud de la portadora (0-255)
        Am       : amplitud de la moduladora (0-255), Am <= Ac recomendado
        fc       : frecuencia de la portadora en Hz
        fm       : frecuencia de la moduladora en Hz (debe ser < fc)
        offset   : desplazamiento DC (0-255), 127 centra la señal en ~1.65 V
        duracion : tiempo total de la señal en segundos
        fs       : tasa de muestreo en muestras/segundo (max. seguro: ~450)
    """
    m = Am / Ac
    pico_max = offset + Ac + Am
    pico_min = offset - Ac - Am

    if m > 1:
        print(f"  ADVERTENCIA: sobremodulacion detectada (m={m:.2f}).")
    if pico_max > 255:
        print(f"  ADVERTENCIA: saturacion en techo ({pico_max:.0f} > 255). "
              f"Reduce amplitudes u offset.")
    if pico_min < 0:
        print(f"  ADVERTENCIA: saturacion en piso ({pico_min:.0f} < 0). "
              f"Aumenta offset o reduce amplitudes.")

    total_muestras = int(duracion * fs)
    t = np.linspace(0, duracion, total_muestras, endpoint=False)

    # Señal AM con offset DC — sin rectificacion
    s = offset + Ac * (1 + (Am / Ac) * np.sin(2 * np.pi * fm * t)) * np.sin(2 * np.pi * fc * t)
    valores = np.clip(s, 0, 255).astype(int)

    print(f"Enviando onda AM con offset: fc={fc}Hz | fm={fm}Hz | "
          f"Ac={Ac} | Am={Am} | offset={offset} | m={m:.2f}")
    print(f"  Rango señal DAC: [{pico_min:.0f}, {pico_max:.0f}]  "
          f"({pico_min/255*3.3:.2f} V – {pico_max/255*3.3:.2f} V)")
    print(f"  fs={fs} sps | muestras={total_muestras} | duracion={duracion}s")
    print(f"  [debug] primeros valores: {list(valores[:6])}")

    conn.reset_input_buffer()
    conn.reset_output_buffer()

    CHUNK = 20
    pausa = CHUNK / fs

    for inicio in range(0, total_muestras, CHUNK):
        bloque = b"\n".join(
            f"SET_PWM:{v}".encode() for v in valores[inicio:inicio + CHUNK]
        ) + b"\n"
        conn.write(bloque)
        conn.flush()
        time.sleep(pausa)

    conn.write(b"SET_PWM:0\n")
    conn.flush()
    print("Onda AM completada. DAC en 0 V.")


try:
    conexion_serial = serial.Serial(PUERTO, BAUDRATE, timeout=1)
    time.sleep(2)   # Espera al reset/boot del ESP32

    if verificar_conexion(conexion_serial):
        enviar_onda_am(conexion_serial, fs=SAMPLE_RATE, offset=OFFSET)
    else:
        print("No se pudo establecer conexión con el ESP32.")

except serial.SerialException as error:
    print(f"Error al abrir el puerto serial: {error}")

finally:
    if 'conexion_serial' in locals() and conexion_serial.is_open:
        conexion_serial.close()
        print("Puerto serial cerrado.")