# Context — Práctica 3: Virtual Instrumentation (ESP32 + Python, no LabVIEW)

## Project
Third practice of the Virtual Instrumentation course. An RGB LED is controlled in two selectable modes, replicating all specifications of `guia_practica3_vir.pdf` but replacing LabVIEW with a Python server and an HTML GUI, and replacing Arduino with an ESP32. The firmware is developed in C++ with **PlatformIO** (Arduino framework).

### Mandatory specifications (from the PDF)
1. The user must be able to choose between two control modes for the RGB LED.
2. **LM35 mode:** the RGB color changes as a function of the temperature signal. The GUI displays the measured temperature and the current LED color. Configurable low/high references (Frío/verde, Ambiente/azul, Caliente/rojo, as in Fig. 2).
3. **Hall mode (3144 + KY-036):** the LED toggles between two colors (magenta/cian) when a magnet or a hand approaches. The GUI displays the state change and the **voltage** on the Hall sensor line.
4. GUI equivalent to the suggested front panels: port selector, tabs (Hall / Temperatura), Start/Pause per mode, global STOP.
5. **RETO (40%):** state machine implemented in `server/app.py` (`StateMachine`): `IDLE`, `HALL_RUN`, `HALL_PAUSE`, `LM35_RUN`, `LM35_PAUSE`, `STOPPED`.

## Architecture
- **ESP32 firmware (PlatformIO, C++/Arduino):** reads sensors (calibrated ADC via `analogReadMilliVolts`), drives the RGB LED with LEDC PWM (1 kHz, 8 bits), and exchanges JSON lines over USB serial at 115200 baud. It is a pure I/O bridge; it holds no control logic.
- **Serial protocol:** ESP32→PC every 100 ms `{"t":24.5,"vh":3.28,"ky":0,"vky":1.20}`; PC→ESP32 `{"cmd":"rgb","r":..,"g":..,"b":..}` or `{"cmd":"off"}`.
- **Python server (Flask + pyserial):** owns the state machine, bridges serial ↔ browser, serves the GUI, streams data by Server-Sent Events (`/events`, 10 Hz) and receives commands (`/cmd`), lists ports (`/ports`).
- **HTML/JS GUI:** tabs, LED lamps, LCD-style readouts, vertical thermometer, live RGB swatch, state badge.

## Project structure (all files created)
```
practica3/
├── firmware/
│   ├── platformio.ini     # esp32dev + Arduino framework + ArduinoJson
│   └── src/
│       └── main.cpp       # ADC reads, PWM RGB, JSON serial protocol
├── server/
│   ├── app.py             # Flask + SerialLink + StateMachine (RETO)
│   ├── templates/
│   │   └── index.html     # Front panel (tabs Hall / LM35)
│   └── static/
│       ├── style.css      # Graphite instrument-panel aesthetic
│       └── main.js        # SSE client, commands, indicator updates
├── requirements.txt       # flask, pyserial
└── context.md
```

## Connection map (ESP32 DevKit, 3.3 V logic)
| Element | Pin (element) | Pin (ESP32) | Notes |
|---|---|---|---|
| LM35 | VCC | VIN (5 V) | LM35 requires ≥ 4 V supply |
| LM35 | VOUT | GPIO34 (ADC1_CH6) | 10 mV/°C; safe for 3.3 V ADC at room temp |
| LM35 | GND | GND | Common ground |
| Hall 3144 | VCC | 3V3 | |
| Hall 3144 | OUT | GPIO35 (ADC1_CH7) | Open collector → 10 kΩ pull-up to 3V3; read as ADC to display voltage |
| Hall 3144 | GND | GND | |
| KY-036 | + | 3V3 | Touch module |
| KY-036 | AO | GPIO32 (ADC1_CH4) | Analog level (displayed in GUI) |
| KY-036 | DO | GPIO33 | Digital touch detection |
| KY-036 | G | GND | |
| RGB LED (common cathode) | R | GPIO25 | 220–330 Ω series resistor |
| RGB LED | G | GPIO26 | 220–330 Ω series resistor |
| RGB LED | B | GPIO27 | 220–330 Ω series resistor |
| RGB LED | Cathode | GND | If common anode: anode → 3V3 and invert duty in `setRGB` |

Rules: only ADC1 pins (32–39) are used because ADC2 conflicts with Wi-Fi; GPIO34/35 are input-only, which is acceptable for sensors. Hall detection threshold: `vh < 1.0 V` (line pulled low) or `ky == 1`.

## Dependencies (Arch Linux)
```bash
# System packages
sudo pacman -S python python-pip picocom

# PlatformIO Core: available as 'platformio-core' in the official repos;
# if not found, install it from the AUR or with pipx (pipx install platformio).
sudo pacman -S platformio-core

# udev rules recommended by PlatformIO for device access
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Serial port permissions (log out/in afterwards)
sudo usermod -aG uucp $USER          # Arch uses 'uucp' (Debian uses 'dialout')

# Option A: Python virtual environment (venv)
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Option B: Conda environment
# On Arch, install miniconda from the AUR (e.g. 'yay -S miniconda3'),
# unlike Debian-based distros where it is installed via the official installer script.
conda create -n practica3 python=3.12
conda activate practica3
pip install -r requirements.txt

# requirements.txt content: flask, pyserial
```

## Commands to run the project
```bash
# 1. Compile and flash the firmware (port auto-detected, usually /dev/ttyUSB0)
cd firmware
pio run                       # Compile only
pio run -t upload             # Compile and flash
pio device monitor -b 115200  # Optional: verify the JSON telemetry stream

# 2. Run the server (env active, from the project root)
python server/app.py          # Serves http://localhost:5000

# 3. Open the GUI
xdg-open http://localhost:5000
```

## Step-by-step plan
1. Assemble the circuit according to the connection map and verify supplies with a multimeter. **(pending)**
2. Install dependencies and flash the firmware with PlatformIO. **(code ready)**
3. Validate telemetry with `pio device monitor`: JSON lines at 10 Hz. **(pending)**
4. Run the server, connect from the GUI (Puerto USB → Conectar). **(code ready)**
5. Test the state machine: Iniciar/Pausar per mode, STOP, invalid transitions rejected. **(routes smoke-tested without hardware)**
6. Calibrate: Hall threshold (1.0 V), temperature references, KY-036 potentiometer.
7. Document evidence (screenshots, state diagram) for the report.

## Notations / rules for this chat
- Language: reply in the language of the last message.
- Code with comments, no emojis.
- OS: Arch Linux.
- Work proceeds one step at a time; do not dump all steps in the conversation.

## Problems / decisions
- MicroPython was replaced by PlatformIO (C++/Arduino) as the firmware implementation; the serial protocol is unchanged.
- The 3144 output is read as ADC (with pull-up) to satisfy the "display the Hall voltage" requirement.
- STOP turns the LED off and enters `STOPPED`; restart is allowed with the Iniciar buttons.

## Current status / next step
- **Status:** all project files created; server routes and state machine smoke-tested without hardware.
- **Next step:** Step 1 — assemble the circuit per the connection map, then flash with `pio run -t upload`.
