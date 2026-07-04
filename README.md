# Virtual Instrumentation repository

This repository compiles the codes used to implement the practices of the subject Virtual Instrumentation for Physics Engineering at the National University of Colombia.
In this course, embedded systems were implemented using virtual instruments programmed in Python and ESP32 microcontrollers as a data acquisition card programmed in C++.

## Practice 1: optical link with amplitude-modulated waves

An analog optical link was built between an infrared LED (transmitter) and a
phototransistor (receiver). The ESP32 generates an amplitude-modulated (AM)
signal on its 8-bit DAC (0–255) — carrier amplitude `Ac`, modulating amplitude
`Am`, carrier frequency `fc`, modulating frequency `fm` and DC offset — drives
the LED through a current-limiting resistor, and simultaneously samples the
demodulated signal from the phototransistor with the 12-bit ADC. Samples are
streamed over UART (230400 baud) to the PC as raw ADC counts to keep the
throughput high enough for the sampling rate.

On the PC side there are three virtual instruments:

- `com_mcu_sample.py` — one-shot capture and plot of the received waveform.
- `com_mcu_mod.py` — commands the ESP32 to generate a specific AM waveform and
  records the response.
- `medir_fs.py` — live acquisition GUI (Tkinter + matplotlib) with sliders that
  tune `Ac`, `Am`, `fc`, `fm` and `offset` on the fly via a `SET_AM` serial
  command. It filters the received signal (Butterworth IIR, zero-phase
  `filtfilt`) and estimates the fundamental frequencies of the carrier and the
  envelope from its spectrum.

Files: `Practica1/esp32_firmware/esp32_firmware.ino` (firmware),
`com_mcu_sample.py`, `com_mcu_mod.py`, `com_mcu_pc.py`, `medir_fs.py`.


## Practice 2: PID control of temperature and angular velocity

Two independent PID loops share a single interface: temperature of a power
resistor (30–35 °C, LM35 sensor) and angular velocity of an N20 DC micromotor
with encoder. The PID and the interface run on the PC; the ESP32 acts only as
I/O — it generates the PWM, reads the LM35 through the ADC, and counts encoder
edges. Both ends talk over the USB serial port with a plain-text protocol.
Only one subsystem is operated at a time.

Two equivalent front-ends are provided:

- **Web GUI** (`server.py` + `index.html` + `styles.css` + `app.js`): FastAPI
  backend with `pyserial-asyncio`, WebSocket telemetry, and a Chart.js dashboard
  with four synchronized plots (measurement vs setpoint, PWM duty, tracking
  error, P/I/D components).
- **Desktop GUI** (`gui.py`): PyQt6 with a reader thread (`serial_link.py`).

Both expose the same controls: port selection, subsystem selection
(temperature/velocity), reference and PID gains (`Kp`, `Ki`, `Kd`), sampling
period `Ts`, plus start/stop/emergency-stop/reset/CSV-export actions.

Hardware map (ESP32): GPIO34 reads the LM35, GPIO25 drives an IRLZ44N MOSFET
that switches a 10 Ω / 5 W heating resistor (6.5 V rail, 15 Hz PWM); GPIO26/27
drive a DRV8833 that powers the N20 motor (~3.3 V rail, 1 kHz PWM) and GPIO32
reads the encoder channel A. See `Practica2/README.md` for the full pinout,
protocol, wiring notes, PID calibration and safety warnings.

Files: `Practica2/firmware_pid_esp32/firmware_pid_esp32.ino`, `server.py`,
`index.html`, `styles.css`, `app.js`, `gui.py`, `serial_link.py`, `pid.py`.