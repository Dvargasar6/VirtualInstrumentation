"""Practica 3 - Virtual Instrumentation server.

Responsibilities:
  1. Serial bridge with the ESP32 (pyserial, JSON lines at 115200 baud).
  2. Control state machine (RETO, 40% of the practice):
       IDLE -> HALL_RUN <-> HALL_PAUSE
       IDLE -> LM35_RUN <-> LM35_PAUSE
       any  -> STOPPED (global STOP; LED off)
  3. HTTP front panel: Flask serves the HTML GUI, streams live data through
     Server-Sent Events (/events) and receives commands (/cmd).
"""

import json
import threading
import time

import serial                          # pyserial: serial port access
from serial.tools import list_ports    # Enumeration of available ports
from flask import Flask, Response, jsonify, render_template, request

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Control constants
# ---------------------------------------------------------------------------
COLOR_COLD    = (0, 0, 255)      # Blue  -> "Frio"     (matches Fig. 2 of the guide)
COLOR_AMBIENT = (0, 255, 0)      # Green   -> "Ambiente"
COLOR_HOT     = (255, 0, 0)      # Red    -> "Caliente"

COLOR_FIELD    = (255, 0, 255)   # Magenta when a magnetic field / touch is present
COLOR_NO_FIELD = (0, 255, 255)   # Cyan when idle (two-color requirement of the guide)

HALL_THRESHOLD_V = 1.0           # 3144 is open collector: line falls near 0 V with field


class SerialLink:
    """Owns the serial port and a background reader thread."""

    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()          # Protects telemetry across threads
        self.telemetry = {"t": None, "vh": None, "ky": 0, "vky": None}
        self.connected = False
        self.port_name = None

    def connect(self, port: str, baud: int = 115200):
        """Open the port and start the reader thread (daemon dies with the app)."""
        self.close()
        self.ser = serial.Serial(port, baud, timeout=1)
        self.port_name = port
        self.connected = True
        threading.Thread(target=self._reader, daemon=True).start()

    def close(self):
        """Close the port if open and mark the link as disconnected."""
        self.connected = False
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.port_name = None

    def _reader(self):
        """Blocking loop: read JSON lines and feed the state machine."""
        while self.connected and self.ser is not None:
            try:
                raw = self.ser.readline()                 # Blocks up to 1 s (timeout)
                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue                              # Timeout or empty line
                data = json.loads(line)
                with self.lock:
                    self.telemetry.update(data)           # Keep the latest sample
                machine.on_telemetry(self.snapshot())     # Run one control step
            except json.JSONDecodeError:
                continue                                  # Skip malformed lines
            except (serial.SerialException, OSError):
                self.connected = False                    # Cable unplugged, etc.

    def send(self, obj: dict):
        """Serialize a command as one JSON line and write it to the ESP32."""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.write((json.dumps(obj) + "\n").encode())
            except (serial.SerialException, OSError):
                self.connected = False

    def snapshot(self) -> dict:
        """Thread-safe copy of the latest telemetry."""
        with self.lock:
            return dict(self.telemetry)


class StateMachine:
    """Implements the RETO state machine that governs the RGB LED."""

    VALID_STATES = {"IDLE", "HALL_RUN", "HALL_PAUSE",
                    "LM35_RUN", "LM35_PAUSE", "STOPPED"}

    def __init__(self, link: SerialLink):
        self.link = link
        self.lock = threading.Lock()
        self.state = "IDLE"
        self.ref_low = 20.0       # "Referencia de temperatura baja" (GUI editable)
        self.ref_high = 30.0      # "Referencia de temperatura alta"
        self.led = (0, 0, 0)      # Last color sent to the ESP32
        self.field = False        # True when magnet/touch detected (Hall mode)
        self.zone = None          # "frio" | "ambiente" | "caliente" (LM35 mode)

    # ------------------------- command handling ---------------------------
    def command(self, cmd: str, payload: dict) -> tuple[bool, str]:
        """Apply a GUI command as a state transition. Returns (ok, message)."""
        with self.lock:
            if cmd == "start_hall":
                self.state = "HALL_RUN"
            elif cmd == "pause_hall":
                if self.state != "HALL_RUN":
                    return False, "pause_hall only valid from HALL_RUN"
                self.state = "HALL_PAUSE"
            elif cmd == "start_lm35":
                self.state = "LM35_RUN"
            elif cmd == "pause_lm35":
                if self.state != "LM35_RUN":
                    return False, "pause_lm35 only valid from LM35_RUN"
                self.state = "LM35_PAUSE"
            elif cmd == "stop":
                self.state = "STOPPED"
                self._set_led((0, 0, 0))          # Global STOP turns the LED off
            elif cmd == "set_refs":
                # Update temperature references, keeping low < high
                low = float(payload.get("low", self.ref_low))
                high = float(payload.get("high", self.ref_high))
                if low >= high:
                    return False, "low reference must be below high reference"
                self.ref_low, self.ref_high = low, high
            else:
                return False, f"unknown command '{cmd}'"
            return True, self.state

    # ------------------------- control loop step --------------------------
    def on_telemetry(self, tel: dict):
        """Called by the serial reader on every sample: compute the LED output."""
        with self.lock:
            if self.state == "HALL_RUN":
                vh = tel.get("vh")
                ky = tel.get("ky", 0)
                if vh is None:
                    return
                # Field present when the 3144 pulls the line low OR the KY-036
                # digital output reports a touch (hand approaching).
                self.field = (vh < HALL_THRESHOLD_V) or (ky == 1)
                self._set_led(COLOR_FIELD if self.field else COLOR_NO_FIELD)

            elif self.state == "LM35_RUN":
                t = tel.get("t")
                if t is None:
                    return
                # Three zones defined by the two GUI references
                if t < self.ref_low:
                    self.zone, color = "frio", COLOR_COLD
                elif t < self.ref_high:
                    self.zone, color = "ambiente", COLOR_AMBIENT
                else:
                    self.zone, color = "caliente", COLOR_HOT
                self._set_led(color)
            # PAUSE/IDLE/STOPPED: hold the current output, no updates.

    def _set_led(self, color: tuple):
        """Send the color to the ESP32 only when it changes (avoids serial spam)."""
        if color != self.led:
            self.led = color
            r, g, b = color
            self.link.send({"cmd": "rgb", "r": r, "g": g, "b": b})

    def snapshot(self) -> dict:
        """State summary streamed to the GUI."""
        with self.lock:
            return {
                "state": self.state,
                "ref_low": self.ref_low,
                "ref_high": self.ref_high,
                "led": self.led,
                "field": self.field,
                "zone": self.zone,
            }


link = SerialLink()
machine = StateMachine(link)

# ---------------------------------------------------------------------------
# HTTP routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    """Serve the front panel."""
    return render_template("index.html")


@app.route("/ports")
def ports():
    """List serial ports so the GUI can populate the 'Puerto USB' selector."""
    return jsonify([p.device for p in list_ports.comports()])


@app.route("/cmd", methods=["POST"])
def cmd():
    """Receive GUI commands: connection management or state transitions."""
    data = request.get_json(force=True)
    name = data.get("cmd", "")

    if name == "connect":
        try:
            link.connect(data.get("port", ""))
            return jsonify(ok=True, msg=f"connected to {link.port_name}")
        except (serial.SerialException, OSError) as exc:
            return jsonify(ok=False, msg=str(exc)), 400

    if name == "disconnect":
        link.close()
        return jsonify(ok=True, msg="disconnected")

    ok, msg = machine.command(name, data)
    return jsonify(ok=ok, msg=msg), (200 if ok else 400)


@app.route("/events")
def events():
    """Server-Sent Events stream: state machine + telemetry at 10 Hz."""
    def stream():
        while True:
            payload = json.dumps({
                "machine": machine.snapshot(),
                "telemetry": link.snapshot(),
                "connected": link.connected,
                "port": link.port_name,
            })
            yield f"data: {payload}\n\n"      # SSE frame format
            time.sleep(0.1)
    return Response(stream(), mimetype="text/event-stream")


if __name__ == "__main__":
    # threaded=True lets Flask serve the SSE stream and commands concurrently
    app.run(host="0.0.0.0", port=5000, threaded=True)
