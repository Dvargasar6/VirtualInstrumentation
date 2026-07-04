// =================================================================
//  Logica del cliente
//  WebSocket al backend FastAPI + actualizacion de Chart.js
// =================================================================

// Numero maximo de puntos visibles en cada grafica (ventana deslizante).
const VENTANA_PUNTOS = 1500;

// Modo actual de la interfaz ("T" o "V"). Se sincroniza con el backend.
let modoActual = "T";

// Estado de conexion. Lo actualiza el backend via WebSocket.
let conectado = false;

// Historico de muestras para exportar a CSV.
const historico = [];

// Tiempo de la primera muestra de la corrida (para escala relativa en X).
let t0 = null;

// Para la tasa efectiva de muestreo (EMA = media movil exponencial).
let tUltimaMuestra = null;
let rateEMA = 0;

// ---------------------------------------------------------------------
//  Inicializacion de las cuatro graficas Chart.js
// ---------------------------------------------------------------------

// Configuracion comun: linea sin animacion, eje X numerico, fondo oscuro.
function configBase(titulo_y) {
  return {
    type: "line",
    data: { datasets: [] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      // Sin animacion: imprescindible para que el repintado a 20 Hz sea fluido.
      animation: false,
      // Sin puntos visibles, solo lineas (mas rapido).
      elements: { point: { radius: 0 } },
      scales: {
        x: {
          type: "linear",
          title: { display: true, text: "Tiempo (s)", color: "#aaa" },
          ticks: { color: "#888" },
          grid:  { color: "#3a3a3a" },
        },
        y: {
          title: { display: true, text: titulo_y, color: "#aaa" },
          ticks: { color: "#888" },
          grid:  { color: "#3a3a3a" },
        },
      },
      plugins: {
        legend: { labels: { color: "#ccc", font: { size: 11 } } },
      },
    },
  };
}

// Grafica 1: medida vs referencia (dos lineas).
const chartMedida = new Chart(document.getElementById("g-medida"),
  configBase("Temperatura (C)"));
chartMedida.data.datasets = [
  { label: "Medida",     data: [], borderColor: "#1f77b4", borderWidth: 2, tension: 0 },
  { label: "Referencia", data: [], borderColor: "#d62728", borderWidth: 2, borderDash: [6,4], tension: 0 },
];

// Grafica 2: ciclo de trabajo (una linea).
const chartDuty = new Chart(document.getElementById("g-duty"),
  configBase("PWM (%)"));
chartDuty.data.datasets = [
  { label: "Duty", data: [], borderColor: "#2ca02c", borderWidth: 2, tension: 0 },
];
chartDuty.options.scales.y.min = 0;
chartDuty.options.scales.y.max = 100;

// Grafica 3: error de seguimiento.
const chartError = new Chart(document.getElementById("g-error"),
  configBase("Error"));
chartError.data.datasets = [
  { label: "Error", data: [], borderColor: "#ff7f0e", borderWidth: 2, tension: 0 },
];

// Grafica 4: componentes del PID (tres lineas).
const chartPID = new Chart(document.getElementById("g-pid"),
  configBase("Magnitud"));
chartPID.data.datasets = [
  { label: "P", data: [], borderColor: "#1f77b4", borderWidth: 2, tension: 0 },
  { label: "I", data: [], borderColor: "#2ca02c", borderWidth: 2, tension: 0 },
  { label: "D", data: [], borderColor: "#d62728", borderWidth: 2, tension: 0 },
];

// ---------------------------------------------------------------------
//  WebSocket con el backend
// ---------------------------------------------------------------------

// Calculamos la URL del WebSocket de forma relativa: si la pagina viene
// de http://hostname:8000, la WS sera ws://hostname:8000/ws.
const wsUrl = (location.protocol === "https:" ? "wss://" : "ws://")
              + location.host + "/ws";
const ws = new WebSocket(wsUrl);

ws.onopen = () => {
  // Al abrirse la conexion, solicitamos la lista de puertos disponibles.
  ws.send(JSON.stringify({accion: "listar_puertos"}));
};

ws.onmessage = (evt) => {
  const msg = JSON.parse(evt.data);
  manejarMensaje(msg);
};

ws.onclose = () => {
  document.getElementById("lbl-conexion").textContent = "Servidor desconectado";
};

// Despachador de mensajes JSON entrantes desde el backend.
function manejarMensaje(msg) {
  if (msg.tipo === "puertos") {
    actualizarSelectorPuertos(msg.puertos);
  }
  else if (msg.tipo === "estado_conexion") {
    conectado = msg.conectado;
    document.getElementById("btn-conectar").textContent =
      conectado ? "Desconectar" : "Conectar";
    document.getElementById("lbl-conexion").textContent =
      conectado ? "Conectado al ESP32" : "Desconectado";
    document.getElementById("btn-iniciar").disabled = !conectado;
  }
  else if (msg.tipo === "monitor") {
    // Lectura recibida fuera del bucle de control: solo actualizamos etiquetas.
    if (msg.modo === modoActual) {
      actualizarEtiquetas(msg.medida, msg.duty,
                          parseFloat(document.getElementById("inp-ref").value) - msg.medida);
    }
  }
  else if (msg.tipo === "muestra") {
    // Muestra completa del bucle de control: graficar y registrar.
    procesarMuestra(msg);
  }
}

function actualizarSelectorPuertos(puertos) {
  const sel = document.getElementById("sel-puerto");
  sel.innerHTML = "";
  puertos.forEach(p => {
    const opt = document.createElement("option");
    opt.value = p; opt.textContent = p;
    sel.appendChild(opt);
  });
}

// ---------------------------------------------------------------------
//  Procesamiento de muestras del bucle de control
// ---------------------------------------------------------------------

function procesarMuestra(m) {
  if (t0 === null) t0 = m.t;
  const tRel = m.t - t0;

  // Anadimos al historico (para exportacion CSV).
  historico.push({
    t: tRel, modo: m.modo, ref: m.referencia, medida: m.medida,
    error: m.error, duty: m.duty, p: m.p, i: m.i, d: m.d,
  });

  // Anadimos los puntos a cada grafica. Chart.js {x, y} acepta eje numerico.
  chartMedida.data.datasets[0].data.push({x: tRel, y: m.medida});
  chartMedida.data.datasets[1].data.push({x: tRel, y: m.referencia});
  chartDuty.data.datasets[0].data.push  ({x: tRel, y: m.duty});
  chartError.data.datasets[0].data.push ({x: tRel, y: m.error});
  chartPID.data.datasets[0].data.push   ({x: tRel, y: m.p});
  chartPID.data.datasets[1].data.push   ({x: tRel, y: m.i});
  chartPID.data.datasets[2].data.push   ({x: tRel, y: m.d});

  // Ventana deslizante: descartamos los puntos mas antiguos.
  [chartMedida, chartDuty, chartError, chartPID].forEach(c => {
    c.data.datasets.forEach(ds => {
      while (ds.data.length > VENTANA_PUNTOS) ds.data.shift();
    });
    // 'none' suprime la animacion del update, repintado inmediato.
    c.update("none");
  });

  // Actualizamos etiquetas.
  actualizarEtiquetas(m.medida, m.duty, m.error);

  // Tasa de muestreo efectiva (media movil exponencial).
  if (tUltimaMuestra !== null) {
    const dt = m.t - tUltimaMuestra;
    if (dt > 0) {
      const inst = 1.0 / dt;
      rateEMA = (rateEMA <= 0) ? inst : 0.9 * rateEMA + 0.1 * inst;
      document.getElementById("lbl-rate").textContent =
        `Ts efectivo: ${(1000/rateEMA).toFixed(0)} ms (${rateEMA.toFixed(1)} Hz)`;
    }
  }
  tUltimaMuestra = m.t;
}

function actualizarEtiquetas(medida, duty, error) {
  const uni = (modoActual === "T") ? "C" : "RPM";
  document.getElementById("lbl-medido").textContent = `${medida.toFixed(2)} ${uni}`;
  document.getElementById("lbl-pwm").textContent    = `${duty.toFixed(1)} %`;
  // toFixed con signo explicito: si error >= 0 ponemos '+'.
  const signo = error >= 0 ? "+" : "";
  document.getElementById("lbl-error").textContent  = `${signo}${error.toFixed(2)} ${uni}`;
}

// ---------------------------------------------------------------------
//  Manejadores de la interfaz
// ---------------------------------------------------------------------

document.getElementById("btn-refrescar").addEventListener("click", () => {
  ws.send(JSON.stringify({accion: "listar_puertos"}));
});

document.getElementById("btn-conectar").addEventListener("click", () => {
  if (conectado) {
    ws.send(JSON.stringify({accion: "desconectar"}));
  } else {
    const puerto = document.getElementById("sel-puerto").value;
    if (!puerto) { alert("Seleccione un puerto."); return; }
    ws.send(JSON.stringify({accion: "conectar", puerto}));
  }
});

// Cambio de modo (radio buttons).
document.querySelectorAll('input[name="modo"]').forEach(rb => {
  rb.addEventListener("change", () => {
    if (!rb.checked) return;
    modoActual = rb.value;
    aplicarPresetModo(modoActual);
    limpiarHistoricoYGraficas();
    ws.send(JSON.stringify({accion: "modo", modo: modoActual}));
    // Reenviamos la configuracion completa para que el backend tenga los valores.
    enviarConfig();
  });
});

// Preset de valores por defecto al cambiar de modo. Coincide con los valores
// que tenia la GUI PyQt anterior, para no romper la familiaridad del usuario.
function aplicarPresetModo(modo) {
  if (modo === "T") {
    document.getElementById("inp-ref").value = 32.5;
    document.getElementById("inp-ref").min   = 30.0;
    document.getElementById("inp-ref").max   = 35.0;
    document.getElementById("inp-ref").step  = 0.5;
    document.getElementById("inp-kp").value = 18.0;
    document.getElementById("inp-ki").value = 1.5;
    document.getElementById("inp-kd").value = 0.5;
    document.getElementById("inp-ts").value = 200;
    document.getElementById("lbl-ref").textContent = "Referencia (C):";
    document.getElementById("titulo-medida").textContent = "Temperatura medida vs referencia";
    chartMedida.options.scales.y.title.text = "Temperatura (C)";
  } else {
    document.getElementById("inp-ref").value = 120;
    document.getElementById("inp-ref").min   = 0;
    document.getElementById("inp-ref").max   = 250;
    document.getElementById("inp-ref").step  = 5;
    document.getElementById("inp-kp").value = 0.10;
    document.getElementById("inp-ki").value = 0.50;
    document.getElementById("inp-kd").value = 0.00;
    document.getElementById("inp-ts").value = 50;
    document.getElementById("lbl-ref").textContent = "Referencia (RPM):";
    document.getElementById("titulo-medida").textContent = "Velocidad medida vs referencia";
    chartMedida.options.scales.y.title.text = "Velocidad (RPM)";
  }
  chartMedida.update("none");
}

// Envia al backend la configuracion completa actual.
function enviarConfig() {
  ws.send(JSON.stringify({
    accion: "config",
    setpoint: parseFloat(document.getElementById("inp-ref").value),
    kp:       parseFloat(document.getElementById("inp-kp").value),
    ki:       parseFloat(document.getElementById("inp-ki").value),
    kd:       parseFloat(document.getElementById("inp-kd").value),
    ts_ms:    parseInt  (document.getElementById("inp-ts").value),
  }));
}

// Cualquier cambio en los inputs sincroniza la configuracion con el backend.
["inp-ref", "inp-kp", "inp-ki", "inp-kd", "inp-ts"].forEach(id => {
  document.getElementById(id).addEventListener("change", enviarConfig);
});

function limpiarHistoricoYGraficas() {
  historico.length = 0;
  t0 = null;
  tUltimaMuestra = null;
  rateEMA = 0;
  [chartMedida, chartDuty, chartError, chartPID].forEach(c => {
    c.data.datasets.forEach(ds => { ds.data = []; });
    c.update("none");
  });
}

document.getElementById("btn-iniciar").addEventListener("click", () => {
  enviarConfig();
  limpiarHistoricoYGraficas();
  ws.send(JSON.stringify({accion: "iniciar"}));
  document.getElementById("btn-detener").disabled = false;
  document.getElementById("btn-iniciar").disabled = true;
});

document.getElementById("btn-detener").addEventListener("click", () => {
  ws.send(JSON.stringify({accion: "detener"}));
  document.getElementById("btn-detener").disabled = true;
  document.getElementById("btn-iniciar").disabled = false;
});

document.getElementById("btn-emergencia").addEventListener("click", () => {
  ws.send(JSON.stringify({accion: "emergencia"}));
  document.getElementById("btn-detener").disabled = true;
  document.getElementById("btn-iniciar").disabled = false;
});

document.getElementById("btn-reset").addEventListener("click", () => {
  ws.send(JSON.stringify({accion: "reset"}));
  limpiarHistoricoYGraficas();
});

// Exportar el historico actual a CSV. Sin servidor: lo construye el navegador
// y lo descarga como blob.
document.getElementById("btn-export").addEventListener("click", () => {
  if (historico.length === 0) {
    alert("No hay datos para exportar.");
    return;
  }
  const lineas = ["t_s,modo,referencia,medida,error,duty_pct,P,I,D"];
  historico.forEach(r => {
    lineas.push([
      r.t.toFixed(4), r.modo, r.ref.toFixed(4), r.medida.toFixed(4),
      r.error.toFixed(4), r.duty.toFixed(4),
      r.p.toFixed(4), r.i.toFixed(4), r.d.toFixed(4)
    ].join(","));
  });
  const blob = new Blob([lineas.join("\n")], {type: "text/csv"});
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url; a.download = "registro.csv";
  a.click();
  URL.revokeObjectURL(url);
});
