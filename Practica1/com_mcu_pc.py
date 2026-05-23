import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import threading

# CONFIG SERIAL
PUERTO   = '/dev/ttyUSB0'
BAUDRATE = 230400

# PARAMETROS AM


# Amplitudes
AMPLITUD_PORTADORA  = 110
AMPLITUD_MODULADORA = 90

# Frecuencias
FRECUENCIA_PORTADORA = 20   # Hz
FRECUENCIA_MODULADORA = 2   # Hz

# Offset DAC
OFFSET = 120

# Tasa envio DAC
SAMPLE_RATE = 500

# Tiempo transmisión
DURACION = 5

# VARIABLES GLOBALES

muestras_adc = []

capturando = True


# VERIFICAR LA CONEXION

def verificar_conexion(conn):

    conn.reset_input_buffer()

    conn.write(b"PING\n")

    t0 = time.time()

    while time.time() - t0 < 2:

        if conn.in_waiting:

            respuesta = conn.readline().decode().strip()

            if respuesta == "PONG":

                print("✓ ESP32 conectado")

                return True

    return False

# HILO RECEPTOR ADC
def recibir_adc(conn):

    global muestras_adc
    global capturando

    while capturando:

        if conn.in_waiting:

            try:

                linea = conn.readline().decode().strip()

                try:

                    valor = float(linea)

                    muestras_adc.append(valor)

                except:
                    pass

            except:
                pass

# TRANSMITIR ONDA AM
def transmitir_am(conn):

    Ac = AMPLITUD_PORTADORA
    Am = AMPLITUD_MODULADORA

    fc = FRECUENCIA_PORTADORA
    fm = FRECUENCIA_MODULADORA

    fs = SAMPLE_RATE

    offset = OFFSET

    total_muestras = int(DURACION * fs)

    t = np.linspace(
        0,
        DURACION,
        total_muestras,
        endpoint=False
    )

  
    # FORMULA AM
    s = offset + Ac * (
        1 + (Am / Ac) * np.sin(2*np.pi*fm*t)
    ) * np.sin(2*np.pi*fc*t)

    valores = np.clip(s, 0, 255).astype(int)

    print("\n========================================")
    print("TRANSMITIENDO AM")
    print("========================================")
    print(f"fc = {fc} Hz")
    print(f"fm = {fm} Hz")
    print(f"Ac = {Ac}")
    print(f"Am = {Am}")
    print(f"fs = {fs}")
    print("========================================\n")

    for v in valores:

        comando = f"SET_PWM:{v}\n"

        conn.write(comando.encode())

        time.sleep(1/fs)

    conn.write(b"SET_PWM:0\n")

    print("✓ Transmisión completada")

# GRAFICA TEMPORAL

def graficar_tiempo(muestras, fs_adc):

    t = np.arange(len(muestras)) / fs_adc

    plt.figure(figsize=(12,5))

    plt.plot(t, muestras)

    # Para ver mejor la AM
    plt.xlim(0, 2)

    plt.title("Señal recibida ADC")

    plt.xlabel("Tiempo [s]")

    plt.ylabel("Voltaje [V]")

    plt.grid()

# FFT
def graficar_fft(muestras, fs_adc):

    N = len(muestras)

    yf = np.fft.fft(muestras)

    xf = np.fft.fftfreq(N, 1/fs_adc)

    magnitud = np.abs(yf) / N

    mitad = N // 2

    plt.figure(figsize=(12,5))

    plt.plot(xf[:mitad], magnitud[:mitad])

    plt.title("Transformada de Fourier")

    plt.xlabel("Frecuencia [Hz]")

    plt.ylabel("Magnitud")

    plt.grid()

    plt.xlim(0, 60)

# MAIN
try:

    conexion = serial.Serial(
        PUERTO,
        BAUDRATE,
        timeout=0.01
    )

    # Esperar reinicio ESP32
    time.sleep(2)
   
    # VERIFICAR CONEXION
    if verificar_conexion(conexion):

       
        # ACTIVAR ADC
        conexion.write(b"START_ADC\n")

        time.sleep(0.2)

        print("✓ ADC streaming activado")

        # HILO ADC

        hilo_adc = threading.Thread(
            target=recibir_adc,
            args=(conexion,)
        )

        hilo_adc.start()

        # TRANSMISION AM
        transmitir_am(conexion)

        time.sleep(1)

        # DETENER ADC
        capturando = False

        conexion.write(b"STOP_ADC\n")

        hilo_adc.join()

        print(f"\nMuestras ADC capturadas: {len(muestras_adc)}")

        # CONVERTIR A NUMPY
        muestras = np.array(muestras_adc)

        # ELIMINAR OFFSET DC
        muestras = muestras - np.mean(muestras)

        # fs ADC APROX

        # delayMicroseconds(200)
        # envio 1 de cada 10 muestras

        fs_adc = 500

        # GRAFICAS
        graficar_tiempo(muestras, fs_adc)

        graficar_fft(muestras, fs_adc)

        plt.show()

    else:

        print("✗ No hubo respuesta del ESP32")

except Exception as e:

    print("ERROR:", e)

finally:

    try:
        conexion.close()
    except:
        pass

    print("Puerto serial cerrado")