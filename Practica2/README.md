# Práctica 2 — Control PID de temperatura y velocidad (Instrumentación Virtual)

Dos lazos de control PID con una misma interfaz: temperatura de una resistencia
de potencia (rango 30–35 °C, sensor LM35) y velocidad de un micromotor DC N20
con encoder. El PID y la interfaz corren en el PC; el ESP32 actúa solo como capa
de entrada/salida (genera la PWM, lee el LM35 por ADC y cuenta los flancos del
encoder). Ambos extremos se comunican por el puerto serie (USB) con un protocolo
de texto. Solo se opera un subsistema a la vez.

Hay dos interfaces equivalentes para operar el sistema: una **aplicación web**
(`server.py` + `index.html`) y una **GUI de escritorio** (`gui.py`, PyQt6).

## Estructura del proyecto

    .
    ├── firmware_pid_esp32/
    │   └── firmware_pid_esp32.ino   Firmware del ESP32
    ├── server.py                    Backend web (FastAPI + pyserial-asyncio)
    ├── index.html                   Frontend web (Chart.js + WebSocket)
    ├── pid.py                       Controlador PID
    ├── gui.py                       GUI de escritorio PyQt6
    ├── serial_link.py               Enlace serie con hilo lector (usado por gui.py)
    ├── requirements.txt             Dependencias de Python
    └── README.md                    Este archivo

## Instalación de dependencias (Arch Linux)

Arch marca su Python como "externally managed" (PEP 668): `pip install` global
está bloqueado. Usa un entorno aislado, con venv o con conda.

### Opción A — venv

    python -m venv pid_env               # crea el entorno virtual
    source pid_env/bin/activate          # lo activa (prefijo "(pid_env)" en el prompt)
    pip install -r requirements.txt      # instala las dependencias dentro del entorno

### Opción B — conda

    conda create -n pid_env python=3.11  # crea el entorno
    conda activate pid_env               # lo activa
    pip install -r requirements.txt      # instala las dependencias

`requirements.txt` instala por defecto todo lo necesario. Si desea solo utilizar la GUI de escritorio o solo la GUI web debe comentar en el archivo las líneas correspondientes a las dependencias de la GUI que no va a utilizar. 

## Ejecución

### GUI web

Con el entorno activado y el ESP32 conectado:

    uvicorn server:app --host 127.0.0.1 --port 8000

Luego abre `http://127.0.0.1:8000` en el navegador. Para acceder desde otra
máquina de la LAN, usa `--host 0.0.0.0`.

En la página: selecciona el puerto y pulsa Conectar, elige el subsistema
(temperatura o velocidad), fija la referencia y las ganancias, y pulsa Iniciar.

### GUI de escritorio

Con el entorno activado y el ESP32 conectado:

    python gui.py

En la ventana: selecciona el puerto y pulsa Conectar, elige el subsistema
(temperatura o velocidad), fija la referencia y las ganancias, y pulsa Iniciar.

### Terminal serie (depuración directa, sin GUI)

Útil para hablar con el ESP32 a mano. Cierra antes cualquier GUI o servidor que
ocupe el puerto.

    arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200

Alternativas habituales en Arch: `picocom -b 115200 /dev/ttyUSB0` o
`screen /dev/ttyUSB0 115200` (en screen, se sale con Ctrl-A seguido de K).


## Firmware: compilación y carga al ESP32

Necesitas `arduino-cli` (desde el AUR: `paru -S arduino-cli`). El acceso a los
puertos serie en Arch lo concede el grupo `uucp` (en Debian/Ubuntu es `dialout`):

    sudo usermod -aG uucp $USER          # reinicia la sesión después de ejecutarlo

No hacen falta drivers del conversor USB-serie: los módulos `cp210x` (chips
CP2102) y `ch341` (chips CH340) ya vienen en el kernel de Arch.

Configuración inicial del núcleo ESP32 (solo la primera vez):

    arduino-cli config init
    arduino-cli config add board_manager.additional_urls \
        https://espressif.github.io/arduino-esp32/package_esp32_index.json
    arduino-cli core update-index
    arduino-cli core install esp32:esp32

Compilar y subir:

    arduino-cli board list               # localiza el puerto, p. ej. /dev/ttyUSB0
    arduino-cli compile --fqbn esp32:esp32:esp32 firmware_pid_esp32
    arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware_pid_esp32

Si la subida no arranca, mantén pulsado el botón BOOT de la placa al empezar.

## Mapa de hardware

Asignación de pines del ESP32:

| Pin ESP32 | Conecta a                                   | Subsistema  |
|-----------|---------------------------------------------|-------------|
| GPIO34    | Salida (OUT) del LM35                        | Temperatura |
| GPIO25    | Resistencia 1 kΩ → Gate del MOSFET IRLZ44N   | Temperatura |
| GPIO26    | IN1 del DRV8833 (PWM)                        | Motor       |
| GPIO27    | IN2 del DRV8833 (dirección, LOW = un sentido)| Motor       |
| GPIO32    | Canal A del encoder (interrupción)           | Motor       |
| GPIO33    | Canal B del encoder (reservado, sin uso)     | Motor       |
| 5V        | Alimentación del LM35                        | Temperatura |
| 3V3       | VCC del encoder                              | Motor       |

### Subsistema de temperatura

- **Sensor LM35:** alimentado del pin 5 V del ESP32; salida (OUT) a GPIO34.
- **Etapa de potencia:** MOSFET IRLZ44N como low-side switch. Gate a GPIO25 a
  través de R = 1 kΩ, con pull-down de 10 kΩ a GND para fijar el gate.
- **Resistencia calefactora:** 10 Ω 5 W, conmutada por el MOSFET.
- **Fuente de potencia:** 6.5 V para la resistencia. Voltaje no mayor a 7 V.

### Subsistema de velocidad

- **Driver DRV8833:** 
  - Control: IN1 → GPIO26 (PWM), IN2 → GPIO27 (LOW para un solo sentido).
  - Salidas: OUT1 → cable rojo del motor, OUT2 → cable blanco.
  - Condensador de 100 µF entre VCC y GND del driver.
- **Motor N20 con encoder**:
  - Negro → 3,3 V del ESP32 (VCC del encoder).
  - Azul → GND común.
  - Amarillo → GPIO32 (canal A).
  - Verde → GPIO33 (canal B, reservado).
- **Fuente del motor:** ≈ 3,3 V al VCC del DRV8833. 

### Fuentes y tierras (resumen)

| Elemento                       | Alimentación                 |
|--------------------------------|------------------------------|
| ESP32                          | USB del PC                   |
| LM35                           | Pin 5 V del ESP32            |
| Resistencia calefactora        | 6,5 V   (No mayor a 7 V)     |
| Motor (vía DRV8833)            | 3,3 V                        |
| Encoder                        | Pin 3,3 V del ESP32          |

**Seguridad (de la guía):** No alimentar la resistencia ni el motor desde los
pines del ESP32, solo señales de control. **Todas las tierras** (ESP32, fuente de
la resistencia, fuente del motor) deben unirse en una **tierra común**.

## Protocolo serie (PC ↔ ESP32), 115200 baudios

PC → ESP32 (texto terminado en `\n`):

    MODE,T     selecciona modo temperatura
    MODE,V     selecciona modo velocidad
    DUTY,xx    fija el ciclo de trabajo (xx entre 0 y 100)
    STOP       pone el ciclo de trabajo a 0
    RESET      duty 0 y reinicia el contador del encoder
    PING       el ESP32 responde PONG
    PCOUNT     responde PCOUNT,<n> con el contador bruto de pulsos (sin reiniciar)

ESP32 → PC:

    T,<°C>,<duty>     telemetría en modo temperatura
    V,<rpm>,<duty>    telemetría en modo velocidad
    READY            al arrancar
    PONG             respuesta a PING
    PCOUNT,<n>       respuesta a PCOUNT

## Calibración de PULSES_PER_REV (en el firmware)

`PULSES_PER_REV` es el número de flancos de subida del canal A por cada vuelta
completa del eje de salida, e incluye la relación de reducción de la caja. Está
calibrado a **207.0** (medido girando el eje de salida diez vueltas a mano,
promediado). Si cambias de motor, recalíbralo: gira el eje exactamente diez
vueltas y divide los pulsos contados entre diez.

## Notas

- Frecuencias PWM: 15 Hz para la resistencia (la planta térmica integra el
  calor) y 1 kHz para el motor.
- Periodo de telemetría del firmware: `SAMPLE_MS = 50` (20 Hz).
- Si la primera medida tarda tras conectar, es normal: al abrir el puerto, la
  señal DTR reinicia el ESP32 y la GUI espera ~2 s a que arranque.