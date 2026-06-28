"""
server.py - Backend de la Practica 2 de Instrumentacion Virtual.

Reemplaza la interfaz PyQt6 + pyqtgraph por un servidor web (FastAPI) que se
controla desde el navegador. La razon de cambiar de arquitectura es que la
implementacion anterior usaba pyserial sobre hilos (un hilo lector + el hilo
de Qt escribiendo), y esa combinacion presenta carreras documentadas en Linux:
las escrituras del hilo principal pueden bloquear las lecturas del hilo lector
indefinidamente, lo que provocaba que la telemetria del ESP32 dejara de llegar
durante toda la fase activa del control.

Aqui toda la entrada/salida del puerto serie ocurre en una unica tarea
asyncio del mismo hilo. No hay dos contextos compitiendo por el objeto
Serial. La biblioteca pyserial-asyncio cede el control cooperativamente
mientras espera bytes, asi que la tarea de control y la WebSocket avanzan
en paralelo logicamente pero sin concurrencia real sobre el puerto.

Dependencias (instalar dentro del entorno virtual del proyecto, p. ej. pid_env):
    pip install fastapi uvicorn pyserial pyserial-asyncio

Ejecucion:
    uvicorn server:app --host 127.0.0.1 --port 8000
    # despues abrir http://127.0.0.1:8000 en el navegador
"""

import asyncio
import json
import time
from typing import Optional

# pyserial-asyncio: capa asyncio sobre pyserial. Expone open_serial_connection,
# que devuelve un par (reader, writer) compatible con los streams de asyncio.
import serial_asyncio
# La biblioteca base pyserial sigue siendo util para enumerar los puertos.
import serial.tools.list_ports

# FastAPI: framework web ligero con soporte nativo de WebSocket.
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

# Reutilizamos el controlador PID existente, sin cambios.
from pid import PID


# ---------------------------------------------------------------------------
#  Constantes del protocolo
# ---------------------------------------------------------------------------
BAUDIOS = 115200            # Velocidad del puerto serie con el ESP32
TS_MS_DEFECTO = 50          # Periodo de muestreo por defecto del bucle PID (ms)


# ---------------------------------------------------------------------------
#  Estado global compartido entre tareas asyncio.
#
#  Como todas las tareas viven en el mismo hilo y solo se intercalan en los
#  'await', no necesitamos bloqueos (Lock) para acceder a estos campos: una
#  tarea solo cede el control en los puntos de espera explicitos.
# ---------------------------------------------------------------------------
class EstadoApp:
    def __init__(self):
        # Streams de asyncio para el puerto serie (None si no esta conectado).
        self.serial_reader: Optional[asyncio.StreamReader] = None
        self.serial_writer: Optional[asyncio.StreamWriter] = None

        # Modo de control: "T" para temperatura, "V" para velocidad.
        self.modo: str = "T"

        # Indica si el bucle PID esta activo (solo entonces se envia DUTY).
        self.corriendo: bool = False

        # Periodo de muestreo del bucle PID, en milisegundos.
        self.ts_ms: int = TS_MS_DEFECTO

        # Controlador PID, inicializado para la planta termica.
        self.pid: PID = PID(kp=18.0, ki=1.5, kd=0.5, setpoint=32.5,
                            out_min=0.0, out_max=100.0)

        # Ultima lectura recibida del ESP32. Diccionario con tipo/valor/duty/t,
        # o None si aun no ha llegado ninguna.
        self.ultima_lectura: Optional[dict] = None

        # WebSocket del unico cliente conectado (None si no hay cliente).
        # Para esta aplicacion local no soportamos varios navegadores a la vez.
        self.cliente_ws: Optional[WebSocket] = None

        # Cola FIFO de comandos pendientes de transmitir al ESP32. Cualquier
        # tarea puede hacer put(); solo la tarea de escritura hace get().
        # Asi todo write() ocurre en un unico contexto, sin carreras.
        self.cola_tx: asyncio.Queue = asyncio.Queue()


estado = EstadoApp()


# ---------------------------------------------------------------------------
#  Tareas asyncio perpetuas
#
#  Cada una es un bucle infinito que se lanza al arrancar el servidor con
#  asyncio.create_task. Conviven cooperativamente: cuando una hace 'await',
#  cede el control al planificador y permite que las otras avancen.
# ---------------------------------------------------------------------------

async def tarea_lectura_serie():
    """
    Lee telemetria del ESP32 y guarda SOLO la ultima lectura disponible.

    Punto critico para control en tiempo real: el ESP32 emite a 20 Hz, pero
    si el backend procesa cada linea con su envio WebSocket asociado, no
    mantiene el ritmo y las lineas se ACUMULAN en el buffer interno. El efecto
    observado es doble: (1) la memoria crece sin limite, y (2) el control lee
    telemetria atrasada varios segundos, de modo que durante la fase activa
    procesa los paquetes viejos del arranque (motor parado, V=0) en lugar de
    los frescos. De ahi que la medida pareciera congelada en cero.

    La solucion es drenar TODO el backlog en cada ciclo y quedarnos solo con
    la ultima linea completa. Un paquete de telemetria atrasado no tiene valor
    en control realimentado: solo importa el mas reciente. Asi el buffer nunca
    crece y la medida es siempre actual.
    """
    buffer = bytearray()  # Acumulador de bytes hasta separar por '\n'
    while True:
        if estado.serial_reader is None:
            await asyncio.sleep(0.05)
            continue
        try:
            # read(n) sobre el StreamReader de asyncio devuelve hasta n bytes
            # tan pronto como haya AL MENOS uno disponible; no espera a juntar
            # los n. Pedimos un bloque grande (4096) para arrastrar de un golpe
            # todo el backlog que se haya acumulado. Esto, combinado con
            # quedarnos solo con la ultima linea (abajo), impide que el retardo
            # crezca y que la memoria se dispare.
            datos = await estado.serial_reader.read(4096)
        except Exception:
            estado.serial_reader = None
            estado.serial_writer = None
            await notificar_estado_conexion(False)
            continue
        if not datos:
            # read() devuelve b"" en EOF (puerto cerrado). Tratamos como
            # desconexion, sin reintentar en bucle ocupado.
            estado.serial_reader = None
            estado.serial_writer = None
            await notificar_estado_conexion(False)
            continue
        buffer.extend(datos)
        # Extraemos todas las lineas completas pero conservamos solo la ULTIMA.
        # Las anteriores son telemetria ya superada: procesarlas solo meteria
        # retardo en el lazo de control.
        ultima_linea = None
        while b"\n" in buffer:
            idx = buffer.index(b"\n")
            linea_bytes = bytes(buffer[:idx])
            del buffer[:idx + 1]
            texto = linea_bytes.decode("ascii", errors="ignore").strip()
            if texto:
                ultima_linea = texto
        if ultima_linea is not None:
            await procesar_linea(ultima_linea)


async def procesar_linea(linea: str):
    """Interpreta una linea recibida del ESP32 segun el protocolo de texto."""
    partes = linea.split(",")
    # Telemetria valida: "T,<valor>,<duty>" o "V,<valor>,<duty>"
    if len(partes) == 3 and partes[0] in ("T", "V"):
        try:
            tipo = partes[0]
            valor = float(partes[1])
            duty = float(partes[2])
        except ValueError:
            return  # Linea malformada por ruido: la descartamos
        estado.ultima_lectura = {
            "tipo": tipo,
            "valor": valor,
            "duty": duty,
            "t": time.perf_counter(),
        }
        # NO enviamos al navegador desde aqui. El envio del monitor en vivo
        # (cuando el control no corre) lo hace la tarea_monitor a 10 Hz, ritmo
        # suficiente para las etiquetas y que no satura el WebSocket. Hacer un
        # send_json por cada linea recibida fue parte del cuello de botella que
        # acumulaba retardo y memoria.
    # Otras lineas del firmware (READY, PONG, PCOUNT,...) se ignoran.


async def tarea_escritura_serie():
    """
    Drena la cola de comandos y los escribe al ESP32.

    Es la unica tarea que llama a serial_writer.write(). Garantizamos asi
    que nunca hay dos tareas escribiendo simultaneamente al mismo writer.
    """
    while True:
        # get() suspende hasta que haya algo en la cola.
        comando = await estado.cola_tx.get()
        if estado.serial_writer is None:
            continue  # Puerto caido: descartamos el comando silenciosamente
        try:
            # El firmware espera ASCII terminado en '\n'.
            estado.serial_writer.write((comando + "\n").encode("ascii"))
            # drain() espera a que el buffer de salida del SO acepte los
            # bytes; cede el control durante esa espera. Imprescindible para
            # que la lectura concurrente pueda avanzar.
            await estado.serial_writer.drain()
        except Exception:
            # Error al escribir: marcamos la conexion como invalida.
            estado.serial_writer = None
            estado.serial_reader = None
            await notificar_estado_conexion(False)


async def tarea_monitor():
    """
    Envia la ultima lectura al navegador a 10 Hz cuando el control NO corre.

    Sirve para que las etiquetas de 'valores actuales' se actualicen aunque
    no se este controlando (modo monitor en vivo). Cuando el control si corre,
    quien envia las muestras es tarea_control, con todos los componentes del
    PID; en ese caso esta tarea no hace nada para no duplicar mensajes.
    """
    while True:
        await asyncio.sleep(0.1)   # 10 Hz, suficiente para etiquetas
        if estado.corriendo:
            continue
        lectura = estado.ultima_lectura
        if lectura is None or estado.cliente_ws is None:
            continue
        try:
            await estado.cliente_ws.send_json({
                "tipo": "monitor",
                "modo": lectura["tipo"],
                "medida": lectura["valor"],
                "duty": lectura["duty"],
            })
        except Exception:
            pass


async def tarea_control():
    """
    Bucle del controlador PID.

    Se ejecuta cada Ts ms cuando estado.corriendo es True. Toma la ultima
    lectura del ESP32, calcula la accion de control, encola un comando DUTY
    para el ESP32 y empuja una muestra completa al navegador para graficar.
    """
    t_prev = time.perf_counter()
    while True:
        # sleep en milisegundos convertidos a segundos.
        await asyncio.sleep(estado.ts_ms / 1000.0)
        if not estado.corriendo:
            # Mientras no corre, mantenemos t_prev al dia para que el primer
            # dt al iniciar sea aproximadamente Ts y no un valor enorme.
            t_prev = time.perf_counter()
            continue
        lectura = estado.ultima_lectura
        if lectura is None:
            continue  # Aun no ha llegado telemetria
        if lectura["tipo"] != estado.modo:
            continue  # Es del modo anterior; esperamos al del modo actual
        ahora = time.perf_counter()
        dt = ahora - t_prev
        t_prev = ahora
        valor = lectura["valor"]
        # El PID devuelve la salida saturada y las tres componentes por separado.
        u, p, i, d = estado.pid.compute(valor, dt)
        # Encolamos el comando para que la tarea de escritura lo envie.
        await estado.cola_tx.put(f"DUTY,{u:.2f}")
        # Enviamos la muestra al navegador para actualizar graficas y etiquetas.
        if estado.cliente_ws is not None:
            try:
                await estado.cliente_ws.send_json({
                    "tipo": "muestra",
                    "t": ahora,
                    "modo": estado.modo,
                    "medida": valor,
                    "referencia": estado.pid.setpoint,
                    "duty": u,
                    "error": estado.pid.setpoint - valor,
                    "p": p, "i": i, "d": d,
                })
            except Exception:
                pass


# ---------------------------------------------------------------------------
#  Utilidades
# ---------------------------------------------------------------------------

async def notificar_estado_conexion(conectado: bool):
    """Avisa al navegador del cambio en el estado de la conexion serie."""
    if estado.cliente_ws is not None:
        try:
            await estado.cliente_ws.send_json({
                "tipo": "estado_conexion",
                "conectado": conectado,
            })
        except Exception:
            pass


async def conectar_serie(puerto: str) -> bool:
    """
    Abre el puerto serie con pyserial-asyncio.

    Devuelve True si la apertura tuvo exito. Espera 2 segundos despues de
    abrir para dar tiempo al ESP32 a completar su reinicio automatico por
    la senal DTR (el chip CP2102 reinicia el ESP32 al abrir el puerto).
    """
    # Si ya habia conexion previa, la cerramos antes de abrir la nueva.
    if estado.serial_writer is not None:
        try:
            estado.serial_writer.close()
        except Exception:
            pass
        estado.serial_writer = None
        estado.serial_reader = None
    try:
        reader, writer = await serial_asyncio.open_serial_connection(
            url=puerto, baudrate=BAUDIOS
        )
    except Exception as e:
        print(f"[server] Error al abrir el puerto {puerto}: {e}")
        return False
    # Espera al reinicio del ESP32. Durante este sleep, otras tareas avanzan.
    await asyncio.sleep(2.0)
    estado.serial_reader = reader
    estado.serial_writer = writer
    return True


# ---------------------------------------------------------------------------
#  FastAPI
# ---------------------------------------------------------------------------

app = FastAPI()


@app.on_event("startup")
async def al_arrancar():
    """Lanza las tareas perpetuas al arrancar el servidor."""
    asyncio.create_task(tarea_lectura_serie())
    asyncio.create_task(tarea_escritura_serie())
    asyncio.create_task(tarea_monitor())
    asyncio.create_task(tarea_control())


@app.get("/", response_class=HTMLResponse)
async def raiz():
    """Sirve el frontend HTML estatico."""
    # Lectura sincrona del archivo en cada peticion: aceptable porque solo
    # se sirve una vez al cargar la pagina, no en cada actualizacion.
    with open("index.html", "r", encoding="utf-8") as f:
        return f.read()


@app.get("/puertos")
async def listar_puertos():
    """Devuelve los puertos serie disponibles en el sistema (en Arch, /dev/ttyUSB*)."""
    return {"puertos": [p.device for p in serial.tools.list_ports.comports()]}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    """
    Punto de entrada del WebSocket.

    Acepta la conexion, registra al cliente, y entra en un bucle que recibe
    mensajes JSON del navegador y los despacha. Al desconectarse el cliente,
    libera el slot.
    """
    await ws.accept()
    estado.cliente_ws = ws
    # Al conectar, informamos del estado actual al navegador.
    await ws.send_json({
        "tipo": "estado_conexion",
        "conectado": estado.serial_writer is not None,
    })
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            await procesar_mensaje_ws(msg, ws)
    except WebSocketDisconnect:
        pass
    finally:
        if estado.cliente_ws is ws:
            estado.cliente_ws = None
        # Si el navegador se va, detenemos el control por seguridad.
        estado.corriendo = False
        await estado.cola_tx.put("STOP")


async def procesar_mensaje_ws(msg: dict, ws: WebSocket):
    """Despacha un mensaje JSON recibido del navegador segun su campo 'accion'."""
    accion = msg.get("accion")

    if accion == "listar_puertos":
        # El cliente solicita refrescar la lista de puertos.
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        await ws.send_json({"tipo": "puertos", "puertos": puertos})

    elif accion == "conectar":
        puerto = msg.get("puerto", "")
        ok = await conectar_serie(puerto)
        await ws.send_json({"tipo": "estado_conexion", "conectado": ok})
        if ok:
            # Comunicamos el modo actual al ESP32 recien arrancado.
            await estado.cola_tx.put(f"MODE,{estado.modo}")

    elif accion == "desconectar":
        estado.corriendo = False
        await estado.cola_tx.put("STOP")
        if estado.serial_writer is not None:
            try:
                estado.serial_writer.close()
            except Exception:
                pass
        estado.serial_writer = None
        estado.serial_reader = None
        await ws.send_json({"tipo": "estado_conexion", "conectado": False})

    elif accion == "modo":
        # Cambio de subsistema (T o V). Reinicia el PID por seguridad.
        nuevo_modo = msg.get("modo", "T")
        if nuevo_modo in ("T", "V"):
            estado.modo = nuevo_modo
            estado.pid.reset()
            await estado.cola_tx.put(f"MODE,{estado.modo}")

    elif accion == "config":
        # Aplicacion de parametros del PID y del periodo de muestreo.
        if "setpoint" in msg:
            estado.pid.setpoint = float(msg["setpoint"])
        if "kp" in msg:
            estado.pid.kp = float(msg["kp"])
        if "ki" in msg:
            estado.pid.ki = float(msg["ki"])
        if "kd" in msg:
            estado.pid.kd = float(msg["kd"])
        if "ts_ms" in msg:
            estado.ts_ms = max(10, int(msg["ts_ms"]))

    elif accion == "iniciar":
        # Reiniciamos el integrador antes de comenzar para evitar arrastrar
        # error acumulado de una corrida anterior.
        estado.pid.reset()
        estado.corriendo = True

    elif accion == "detener":
        estado.corriendo = False
        await estado.cola_tx.put("STOP")

    elif accion == "emergencia":
        # Doble STOP por redundancia, para no depender de un unico paquete.
        estado.corriendo = False
        await estado.cola_tx.put("STOP")
        await estado.cola_tx.put("STOP")
        estado.pid.reset()

    elif accion == "reset":
        estado.pid.reset()
        await estado.cola_tx.put("RESET")