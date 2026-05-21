import serial
import time
import numpy as np
import math

PUERTO = '/dev/ttyUSB0'
BAUDRATE = 115200
TIMEOUT = 2  # seconds to wait for a response

def verificar_conexion(conexion: serial.Serial) -> bool:
    """
    Sends a PING to the ESP32 and waits for a PONG response.
    The ESP32 firmware must respond with 'PONG' when it receives 'PING'.
    """
    print("Verificando conexión con el ESP32...")
    conexion.write("PING\n".encode('utf-8'))
    
    tiempo_inicio = time.time()
    while (time.time() - tiempo_inicio) < TIMEOUT:

        # Se verifica si hay bytes esperando en el bufer de entrada de la computadora.
        if conexion.in_waiting > 0:
            respuesta = conexion.readline().decode('utf-8').strip()
            if respuesta == "PONG":
                print("✓ Conexión verificada: ESP32 respondió correctamente.")
                return True
            else:
                print(f"Respuesta inesperada: '{respuesta}'")
    
    print("✗ Sin respuesta del ESP32. Verifique la conexión.")
    return False


def enviar_comando(conexion: serial.Serial, comando: str) -> str | None:
    """
    Sends a command and waits for a response.
    Returns the response text or None if timeout.
    """
    conexion.write(f"{comando}\n".encode('utf-8'))
    print(f"Comando enviado: '{comando}'")
    
    tiempo_inicio = time.time()
    while (time.time() - tiempo_inicio) < TIMEOUT:
        if conexion.in_waiting > 0:
            return conexion.readline().decode('utf-8').strip()
    
    print("Timeout: sin respuesta al comando.")
    return None


def enviar_onda_seno(conn, cycles=3, steps=10000, delay=0.02):
    """Sends a sine wave as individual PWM values from the PC."""
    print("Enviando onda seno...")
    for i in range(steps * cycles):
        angle = (2 * np.pi * i) / steps
        value = int((np.sin(angle) + 1) * 127.5)  # maps 0-255
        #value = 3.3*np.sin(angle)
        # Send without waiting for response — keeps timing tight
        conn.write(f"SET_PWM:{value}\n".encode('utf-8'))
        conn.flush()

        
        time.sleep(delay)
    #conn.write("SET_PWM:0\n".encode('utf-8'))
    #conn.flush()
    print("Onda seno completada.")


# ---------------------------------------------------------------------------
# PARÁMETROS DE LA ONDA AM
# ---------------------------------------------------------------------------
# Amplitud de la portadora (0–255). Determina el brillo máximo del LED.
# Para evitar saturación, Ac + Am debe ser <= 255.
AMPLITUD_PORTADORA  = 200   # Ac
 
# Amplitud de la moduladora (0–255). Controla la profundidad de modulación.
# El índice de modulación es m = Am / Ac. Mantener m <= 1 (Am <= Ac)
# para evitar sobremodulación y distorsión.
AMPLITUD_MODULADORA = 100   # Am
 
# Frecuencia de la portadora en Hz (ciclos por segundo lógico).
# Valores más altos → oscilación más rápida del brillo.
FRECUENCIA_PORTADORA  = 20  # fc [Hz]
 
# Frecuencia de la moduladora en Hz.
# Debe ser menor que fc para que la envolvente sea visible.
FRECUENCIA_MODULADORA = 2   # fm [Hz]
# ---------------------------------------------------------------------------
 
 
def enviar_onda_am(
    conn      : serial.Serial,
    Ac        : float = AMPLITUD_PORTADORA,
    Am        : float = AMPLITUD_MODULADORA,
    fc        : float = FRECUENCIA_PORTADORA,
    fm        : float = FRECUENCIA_MODULADORA,
    duracion  : float = 60.0,
    steps     : int   = 1000,
    delay     : float = 0.02,
):
    """
    Envía una onda AM (modulación en amplitud) como valores PWM (0–255).
 
    Fórmula:
        s(t) = Ac * [1 + (Am/Ac) * sin(2*pi*fm*t)] * sin(2*pi*fc*t)
 
    El resultado se rectifica con abs() y se limita a 0–255 para que el LED
    siempre reciba un valor positivo válido.
 
    Parámetros:
        Ac       : amplitud de la portadora (0–255)
        Am       : amplitud de la moduladora (0–255), Am <= Ac recomendado
        fc       : frecuencia de la portadora en Hz
        fm       : frecuencia de la moduladora en Hz (debe ser < fc)
        duracion : tiempo total de transmisión en segundos
        steps    : muestras totales a generar (más = más suave)
        delay    : tiempo entre muestras en segundos
    """
    m = Am / Ac
    if m > 1:
        print(f"  ADVERTENCIA: sobremodulación detectada (m = {m:.2f}). "
              f"Considera reducir Am o aumentar Ac.")
 
    total_pasos = int(duracion / delay)
    print(f"Enviando onda AM: fc={fc}Hz, fm={fm}Hz, Ac={Ac}, Am={Am}, m={m:.2f}")
    print(f"  Duración: {duracion}s | Pasos: {total_pasos} | delay: {delay}s")
 
    conn.reset_input_buffer()
    conn.reset_output_buffer()
 
    for i in range(total_pasos):
        t = i * delay  # tiempo actual en segundos
 
        # Señal AM: portadora modulada en amplitud
        s = Ac * (1 + (Am / Ac) * np.sin(2 * np.pi * fm * t)) * np.sin(2 * np.pi * fc * t)
 
        # Rectificación y mapeo a 0–255
        value = int(np.clip(np.abs(s), 0, 255))
 
        conn.write(f"SET_PWM:{value}\n".encode('utf-8'))
        conn.flush()
 
        if i < 8:
            print(f"  [debug] t={t:.3f}s -> SET_PWM:{value}")
 
        time.sleep(delay)
 
    conn.write("SET_PWM:0\n".encode('utf-8'))
    conn.flush()
    print("Onda AM completada. LED apagado.")


try:
    conexion_serial = serial.Serial(PUERTO, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to boot after reset 

    #if verificar_conexion(conexion_serial):
    #    respuesta = enviar_comando(conexion_serial, "PING")
    #    if respuesta:
    #        print(f"Respuesta del ESP32: '{respuesta}'")
    
    if verificar_conexion(conexion_serial):
        #respuesta = enviar_comando(conexion_serial, "PING")
        #enviar_onda_seno(conexion_serial, cycles=3, steps=100, delay=0.02)
        enviar_onda_am(conexion_serial)
    else:
        print("No respuesta")


except serial.SerialException as error:
    print(f"Error al abrir el puerto serial: {error}")

finally:
    if 'conexion_serial' in locals() and conexion_serial.is_open:
        conexion_serial.close()
        print("Puerto serial cerrado.")