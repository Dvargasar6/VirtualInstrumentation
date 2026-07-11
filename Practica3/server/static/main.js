// Practica 3 front-panel logic.
// Responsibilities: populate the port selector, send commands to /cmd,
// consume the /events SSE stream and refresh every indicator.

// Shorthand for document.querySelector
const $ = (sel) => document.querySelector(sel);

// ---------------------------------------------------------------------------
// Command helper: POST a JSON body to /cmd and show the server reply
// ---------------------------------------------------------------------------
async function sendCmd(body) {
  try {
    const res = await fetch("/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const data = await res.json();
    $("#msg").textContent = data.msg || "";   // Status line at the bottom
    return data.ok;
  } catch (err) {
    $("#msg").textContent = "Servidor no disponible: " + err.message;
    return false;
  }
}

// ---------------------------------------------------------------------------
// Serial port selector
// ---------------------------------------------------------------------------
async function loadPorts() {
  const res = await fetch("/ports");          // The server enumerates the ports
  const ports = await res.json();
  const sel = $("#port-select");
  sel.innerHTML = "";                         // Clear previous options
  if (ports.length === 0) {
    sel.append(new Option("N/A", ""));        // Same placeholder as the guide
    return;
  }
  ports.forEach((p) => sel.append(new Option(p, p)));
}

$("#btn-refresh").addEventListener("click", loadPorts);
$("#btn-connect").addEventListener("click", () =>
  sendCmd({ cmd: "connect", port: $("#port-select").value })
);

// ---------------------------------------------------------------------------
// Tabs: show one panel at a time (Hall / temperature)
// ---------------------------------------------------------------------------
document.querySelectorAll(".tab").forEach((tab) => {
  tab.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((t) => t.classList.remove("active"));
    document.querySelectorAll(".panel").forEach((p) => p.classList.remove("active"));
    tab.classList.add("active");
    $("#" + tab.dataset.panel).classList.add("active");
  });
});

// ---------------------------------------------------------------------------
// Mode and STOP buttons -> state machine transitions
// ---------------------------------------------------------------------------
$("#btn-start-hall").addEventListener("click", () => sendCmd({ cmd: "start_hall" }));
$("#btn-pause-hall").addEventListener("click", () => sendCmd({ cmd: "pause_hall" }));
$("#btn-start-lm35").addEventListener("click", () => sendCmd({ cmd: "start_lm35" }));
$("#btn-pause-lm35").addEventListener("click", () => sendCmd({ cmd: "pause_lm35" }));
$("#btn-stop").addEventListener("click", () => sendCmd({ cmd: "stop" }));

// Temperature references: push both values on any change
function pushRefs() {
  sendCmd({
    cmd: "set_refs",
    low: parseFloat($("#ref-low").value),
    high: parseFloat($("#ref-high").value),
  });
}
$("#ref-low").addEventListener("change", pushRefs);
$("#ref-high").addEventListener("change", pushRefs);

// ---------------------------------------------------------------------------
// Indicator helpers
// ---------------------------------------------------------------------------
// Turn a lamp element on/off by toggling its 'on'/'off' classes
function setLamp(id, on) {
  const el = $(id);
  el.classList.toggle("on", on);
  el.classList.toggle("off", !on);
}

// Human-readable name of the current LED color
function ledName([r, g, b]) {
  const map = {
    "0,0,0": "apagado",
    "255,0,0": "rojo (caliente)",
    "0,255,0": "verde (frío)",
    "0,0,255": "azul (ambiente)",
    "255,0,255": "magenta (con campo)",
    "0,255,255": "cian (sin campo)",
  };
  return map[[r, g, b].join(",")] || `rgb(${r}, ${g}, ${b})`;
}

// ---------------------------------------------------------------------------
// SSE stream: one update every 100 ms from /events
// ---------------------------------------------------------------------------
const source = new EventSource("/events");

source.onmessage = (evt) => {
  const { machine, telemetry, connected } = JSON.parse(evt.data);

  // Connection lamp next to the Conectar button
  setLamp("#conn-lamp", connected);

  // ---- Hall panel ----
  if (telemetry.vh !== null) $("#vhall").textContent = telemetry.vh.toFixed(2);
  if (telemetry.vky !== null) $("#vky").textContent = telemetry.vky.toFixed(2);
  setLamp("#lamp-field", machine.field);
  setLamp("#lamp-nofield", !machine.field);

  // ---- LM35 panel ----
  if (telemetry.t !== null) {
    $("#temp").textContent = telemetry.t.toFixed(1);
    // Thermometer fill mapped from 0-100 C to 0-100 % height
    const pct = Math.min(100, Math.max(0, telemetry.t));
    $("#thermo-fill").style.height = pct + "%";
  }
  setLamp("#lamp-cold", machine.zone === "frio");
  setLamp("#lamp-amb", machine.zone === "ambiente");
  setLamp("#lamp-hot", machine.zone === "caliente");

  // ---- Footer: LED monitor and state badge ----
  const [r, g, b] = machine.led;
  const swatch = $("#led-swatch");
  swatch.style.background = `rgb(${r}, ${g}, ${b})`;
  swatch.style.boxShadow =
    r + g + b > 0 ? `0 0 14px 3px rgba(${r}, ${g}, ${b}, .5)` : "none";
  $("#led-name").textContent = ledName(machine.led);
  $("#state-badge").textContent = machine.state;

  // Enable Pausar only while its mode is running (LabVIEW-like behavior)
  $("#btn-pause-hall").disabled = machine.state !== "HALL_RUN";
  $("#btn-pause-lm35").disabled = machine.state !== "LM35_RUN";
};

// Initial population of the port list
loadPorts();
