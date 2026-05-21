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


def enviar_onda_seno(conn, cycles=3, steps=100, delay=0.02):
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

try:
    conexion_serial = serial.Serial(PUERTO, BAUDRATE, timeout=1)
    time.sleep(2)  # Wait for ESP32 to boot after reset 

    #if verificar_conexion(conexion_serial):
    #    respuesta = enviar_comando(conexion_serial, "PING")
    #    if respuesta:
    #        print(f"Respuesta del ESP32: '{respuesta}'")
    
    if verificar_conexion(conexion_serial):
        #respuesta = enviar_comando(conexion_serial, "PING")
        enviar_onda_seno(conexion_serial, cycles=3, steps=100, delay=0.02)

    else:
        print("No respuesta")


except serial.SerialException as error:
    print(f"Error al abrir el puerto serial: {error}")

finally:
    if 'conexion_serial' in locals() and conexion_serial.is_open:
        conexion_serial.close()
        print("Puerto serial cerrado.")