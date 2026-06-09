"""
Enlace serie con el ESP32 para la Practica 2 de Instrumentacion Virtual.

Abre el puerto serie con pyserial y lanza un hilo en segundo plano que lee
continuamente las lineas de telemetria del ESP32, las interpreta y guarda la
ultima lectura disponible de forma segura entre hilos. El bucle de control de
la interfaz (en el hilo principal) consulta esa ultima lectura cada periodo de
muestreo, sin bloquearse esperando datos.

Protocolo (debe coincidir EXACTAMENTE con el firmware del ESP32):
  PC    -> ESP32 :  MODE,T | MODE,V | DUTY,xx | STOP | RESET | PING
  ESP32 -> PC    :  T,<C>,<duty> | V,<rpm>,<duty> | READY | PONG
"""

import threading                  # Para el hilo lector en segundo plano
import time
import serial                     # pyserial: acceso al puerto serie
import serial.tools.list_ports    # Para enumerar los puertos serie del sistema


def listar_puertos():
    """
    Devuelve una lista con los nombres de los puertos serie disponibles.
    En Arch Linux suelen ser '/dev/ttyUSB0' (chips CP2102) o '/dev/ttyACM0'.
    """
    # comports() devuelve objetos con atributos .device y .description
    return [p.device for p in serial.tools.list_ports.comports()]


class SerialLink:
    def __init__(self):
        self._ser = None                  # Objeto Serial de pyserial
        self._reader = None               # Hilo lector
        self._running = False             # Bandera de ejecucion del hilo
        self._lock = threading.Lock()     # Protege el acceso a la ultima lectura
        # Ultima lectura recibida: tupla (tipo, valor, duty, t_recepcion) o None
        self._latest = None

    def conectar(self, puerto, baudios=115200, timeout=1.0):
        """
        Abre el puerto y arranca el hilo lector.
        Lanza una excepcion (serial.SerialException) si el puerto no se puede abrir.

        Nota: al abrir el puerto, el ESP32 suele reiniciarse por la senal DTR;
        por eso esperamos ~2 s antes de dar la conexion por buena. Durante esa
        espera la interfaz queda congelada un instante.
        """
        # Abrimos el puerto serie con el timeout indicado para las lecturas
        self._ser = serial.Serial(puerto, baudios, timeout=timeout)
        time.sleep(2.0)                 # Espera al reinicio automatico del ESP32
        self._ser.reset_input_buffer()  # Descarta datos basura acumulados al abrir
        # Arrancamos el hilo lector como 'daemon' para que muera al cerrar el programa
        self._running = True
        self._reader = threading.Thread(target=self._bucle_lectura, daemon=True)
        self._reader.start()

    def desconectar(self):
        """Detiene el hilo lector y cierra el puerto de forma ordenada."""
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.5)   # Espera a que el hilo termine
            self._reader = None
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._ser = None
        with self._lock:
            self._latest = None

    def conectado(self):
        """Indica si el puerto esta abierto."""
        return self._ser is not None and self._ser.is_open

    def enviar(self, comando):
        """
        Envia un comando de texto al ESP32 anadiendo el salto de linea final.
        Si el puerto no esta abierto, no hace nada.
        """
        if self.conectado():
            try:
                # encode() convierte el texto a bytes; el firmware espera ASCII + '\n'
                self._ser.write((comando + "\n").encode("ascii"))
            except (serial.SerialException, OSError):
                pass  # Si el puerto se cae al escribir, lo ignoramos silenciosamente

    def ultima_lectura(self):
        """
        Devuelve la ultima lectura recibida como (tipo, valor, duty, t) o None.
          tipo  : 'T' (temperatura) o 'V' (velocidad)
          valor : grados Celsius o RPM
          duty  : ciclo de trabajo que el ESP32 reporta estar aplicando (%)
          t     : marca de tiempo de recepcion (time.perf_counter)
        """
        with self._lock:
            return self._latest

    # ----------------------------------------------------------------------
    #  Metodos internos (ejecutados por el hilo lector)
    # ----------------------------------------------------------------------
    def _bucle_lectura(self):
        """Bucle del hilo: lee lineas y actualiza la ultima lectura."""
        while self._running:
            try:
                # readline() lee bytes hasta encontrar '\n' o agotar el timeout.
                # decode() los pasa a texto; errors='ignore' descarta bytes invalidos.
                linea = self._ser.readline().decode("ascii", errors="ignore").strip()
            except (serial.SerialException, OSError):
                break   # Si el puerto se cae, salimos del hilo
            if not linea:
                continue
            self._procesar_linea(linea)

    def _procesar_linea(self, linea):
        """Interpreta una linea de telemetria del ESP32 y guarda el dato util."""
        partes = linea.split(",")
        # Telemetria valida: "T,<valor>,<duty>" o "V,<valor>,<duty>"
        if len(partes) == 3 and partes[0] in ("T", "V"):
            try:
                tipo = partes[0]
                valor = float(partes[1])
                duty = float(partes[2])
            except ValueError:
                return   # Linea malformada: la descartamos
            with self._lock:
                self._latest = (tipo, valor, duty, time.perf_counter())
        # Las lineas "READY" y "PONG" no llevan dato de medida; aqui se ignoran.
