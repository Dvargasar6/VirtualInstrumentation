# Práctica 2 — Control PID de temperatura y velocidad (Instrumentación Virtual)

Software completo para implementar dos lazos de control PID (temperatura de una
resistencia de potencia y velocidad de un motor DC N20) usando un ESP32 como
capa de entrada/salida y un PC con Python como controlador e interfaz.

El PID y la interfaz corren en el PC. El ESP32 solo genera la PWM, lee el LM35
por ADC y cuenta los flancos del encoder. Ambos extremos se comunican por el
puerto serie (USB) con un protocolo de texto.

## Estructura del proyecto

    .
    ├── firmware_pid_esp32/
    │   └── firmware_pid_esp32.ino   Firmware del ESP32 (Arduino, núcleo 3.x)
    ├── pid.py                       Controlador PID (sin dependencias)
    ├── serial_link.py               Enlace serie con hilo lector
    ├── gui.py                       Interfaz PyQt6 + bucle de control (punto de entrada)
    ├── requirements.txt             Dependencias de Python
    └── README.md                    Este archivo

## Requisitos e instalación (Arch Linux)

### Lado PC (Python)

Arch marca su Python como "externally managed" (PEP 668), por lo que `pip
install` global está bloqueado. Usa un entorno virtual, que además aísla las
dependencias de la práctica:

    python -m venv venv                  # crea el entorno virtual
    source venv/bin/activate             # lo activa (deja un prefijo "(venv)" en el prompt)
    pip install -r requirements.txt      # instala las dependencias dentro del entorno

Alternativa con paquetes del sistema (si prefieres no usar venv):

    sudo pacman -S python-pyserial python-pyqt6 python-numpy
    # python-pyqtgraph puede estar solo en el AUR:  paru -S python-pyqtgraph

Instala PyQt6 (no PyQt5): el código usa enumerados encapsulados propios de PyQt6
y pyqtgraph detecta automáticamente el binding instalado.

### Lado microcontrolador (firmware)

Necesitas `arduino-cli` (desde el AUR: `paru -S arduino-cli`) y el núcleo del
ESP32. Detalle propio de Arch: el acceso a los puertos serie lo concede el grupo
`uucp` (en distribuciones Debian es `dialout`):

    sudo usermod -aG uucp $USER          # reinicia la sesión después de ejecutarlo

No hace falta instalar drivers del conversor USB-serie: los módulos `cp210x`
(chips CP2102) y `ch341` (chips CH340) vienen en el kernel de Arch.

## Cargar el firmware

    arduino-cli config init
    arduino-cli config add board_manager.additional_urls \
        https://espressif.github.io/arduino-esp32/package_esp32_index.json
    arduino-cli core update-index
    arduino-cli core install esp32:esp32

    arduino-cli board list                                                   # localiza el puerto, p.ej. /dev/ttyUSB0
    arduino-cli compile --fqbn esp32:esp32:esp32 firmware_pid_esp32 && arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware_pid_esp32

Si la subida no arranca, mantén pulsado el botón BOOT de la placa al empezar.

## Ejecutar la interfaz

Con el entorno virtual activado y el ESP32 conectado:

    python gui.py

En la ventana: selecciona el puerto y pulsa Conectar, elige el subsistema
(temperatura o velocidad), fija la referencia y las ganancias, y pulsa Iniciar.

## Cableado

Asignación de pines del ESP32 (figuras 3 y 5 del preinforme):

| Pin ESP32 | Conecta a                                  | Subsistema  |
|-----------|--------------------------------------------|-------------|
| GPIO34    | Salida (OUT) del LM35                       | Temperatura |
| GPIO25    | Resistencia de 1 kΩ → base del TIP122       | Temperatura |
| GPIO26    | IN1 del DRV8833                             | Motor       |
| GPIO27    | IN2 del DRV8833                             | Motor       |
| GPIO32    | Canal A del encoder                         | Motor       |
| GPIO33    | Canal B del encoder (reservado, sin uso)    | Motor       |

- El LM35 se alimenta con V1 = 6 V.
- La resistencia de 22 Ω/3 W se alimenta desde una fuente independiente V2
  (5 V a 6 V según el cálculo del preinforme), conmutada por el TIP122.
- El motor y su encoder se alimentan desde otra fuente independiente vía DRV8833.

Reglas de seguridad (de la guía): nunca alimentes la resistencia ni el motor
desde los pines del ESP32, solo señales de control; y todas las tierras (ESP32,
V1, V2, fuente del motor) deben unirse en una tierra común.

## Protocolo serie (PC ↔ ESP32), 115200 baudios

PC → ESP32 (texto terminado en `\n`):

    MODE,T     selecciona modo temperatura
    MODE,V     selecciona modo velocidad
    DUTY,xx    fija el ciclo de trabajo (xx entre 0 y 100)
    STOP       pone el ciclo de trabajo a 0
    RESET      duty 0 y reinicia el contador del encoder
    PING       el ESP32 responde PONG

ESP32 → PC:

    T,<°C>,<duty>     telemetría en modo temperatura
    V,<rpm>,<duty>    telemetría en modo velocidad
    READY            al arrancar
    PONG             respuesta a PING

## Calibración obligatoria: PULSES_PER_REV (en el firmware)

`PULSES_PER_REV` es el número de flancos de subida del canal A por cada vuelta
completa del eje de salida, e incluye la relación de reducción de la caja:

    PULSES_PER_REV = PPR_del_encoder_por_canal × relación_de_reducción

Si no tienes la hoja de datos exacta, mídelo: en modo velocidad, gira el eje de
salida exactamente diez vueltas a mano y observa cuántos pulsos cuenta el sistema
(divide entre diez). Las RPM serán incorrectas hasta que pongas el valor real;
el valor por defecto (350) es solo un punto de partida típico para un N20.

## Sintonización del PID

Las ganancias por defecto de temperatura provienen del bosquejo del preinforme.
Las del motor son un punto de partida que casi seguro habrá que ajustar.

Procedimiento manual recomendado (método heurístico):

1. Pon Ki = 0 y Kd = 0. Sube Kp hasta obtener una respuesta rápida con una
   oscilación leve y sostenida.
2. Reduce Kp ligeramente y sube Ki para eliminar el error de estado estacionario.
3. Sube Kd con cuidado para amortiguar el sobreimpulso. En el motor, mantén Kd
   bajo o nulo si el ruido del encoder se amplifica demasiado.

Puedes modificar las ganancias mientras el sistema corre y observar el efecto en
las cuatro gráficas. La gráfica de componentes P/I/D ayuda a ver el windup.

## Problemas conocidos a verificar en hardware

Estos puntos son los candidatos más probables a requerir ajuste cuando pruebes:

- PULSES_PER_REV sin calibrar: las RPM mostradas no coincidirán con la realidad.
- Ganancias del motor: el lazo puede quedar lento, oscilante o inestable hasta
  ajustarlas. Las de temperatura suelen ser un punto de partida razonable.
- Lectura del LM35: a 30–35 °C entrega 300–350 mV, cerca del extremo bajo del
  ADC. Si la temperatura medida es ruidosa o sesgada, sube el promediado en el
  firmware o reduce la atenuación del ADC del LM35.
- Encoder sin pulsos: revisa la alimentación del encoder (3,3 V o 5 V según
  versión) y la tierra común; el firmware usa pull-up interno en el canal A.
- Puerto serie: confirma el nombre real (`/dev/ttyUSB0` vs `/dev/ttyACM0`) y la
  pertenencia al grupo `uucp`.
- Reinicio del ESP32 al conectar: la GUI espera ~2 s tras abrir el puerto; si la
  primera medida tarda, es normal.
