# Contexto del proyecto — Práctica 2 de Instrumentación Virtual (continuación)

Documento de traspaso para continuar en una conversación nueva con contexto
limpio. Adjuntar también, si es posible: la guía del profesor
(`Practica2_VIR.pdf`), el preinforme (`Practica_2_Instrumentacion_Virtual.pdf`),
y los archivos de código actuales (`server.py`, `index.html`, `pid.py`,
`firmware_pid_esp32.ino`).

## Resumen del proyecto

Dos lazos de control PID con una misma interfaz: control de **temperatura** de
una resistencia de potencia (rango 30–35 °C, sensor LM35) y control de
**velocidad** de un micromotor DC N20 con encoder de cuadratura (driver
DRV8833). El PID y la interfaz corren en el PC; el ESP32 actúa solo como capa
de entrada/salida (PWM, ADC del LM35, conteo de flancos del encoder). Solo se
opera un subsistema a la vez desde la GUI, pero ambos están cableados a la vez
físicamente.

## Arquitectura de la GUI (estable)

La GUI original era PyQt6 + PyQtGraph (`gui.py` + `serial_link.py`). Se abandonó
por un problema de comunicación serie y se reescribió como aplicación web:

- **`server.py`**: backend FastAPI + pyserial-asyncio + uvicorn. Toda la I/O
  del puerto serie ocurre en una única tarea asyncio (sin hilos). Habla con el
  navegador por WebSocket.
- **`index.html`**: frontend de una sola página, Chart.js (desde CDN), WebSocket
  nativo. Cuatro gráficas en vivo, exportación a CSV en el navegador.

`gui.py` y `serial_link.py` quedan obsoletos (referencia histórica). `pid.py` se
reutiliza sin cambios.

### Ejecución de la GUI

```
source ~/Development/VirtualInstrumentation/Practica2/pid_env/bin/activate
pip install fastapi "uvicorn[standard]" pyserial-asyncio   # solo la primera vez
uvicorn server:app --host 127.0.0.1 --port 8000
# abrir http://127.0.0.1:8000 en el navegador
```

Para acceso desde otra máquina de la LAN: `--host 0.0.0.0`.

### Protocolo serie (115200 baudios)

PC → ESP32, terminado en `\n`:
```
MODE,T | MODE,V | DUTY,xx | STOP | RESET | PING | PCOUNT
```
ESP32 → PC:
```
T,<°C>,<duty> | V,<rpm>,<duty> | READY | PONG | PCOUNT,<n>
```

`PCOUNT` responde `PCOUNT,<n>` con el contador bruto de pulsos SIN reiniciarlo
(sirvió para calibrar `PULSES_PER_REV`).

## PROBLEMA DE LAS RPM EN LA GUI: RESUELTO

**Síntoma histórico:** al operar el subsistema de velocidad desde la GUI, la
medida de RPM permanecía en 0.0; el PID veía medida=0, el integrador saturaba y
el duty subía a 100 % de forma permanente. El motor giraba físicamente pero la
GUI no lo reflejaba. Además, al pulsar STOP el motor no se detenía y a veces la
medida saltaba a ~260 RPM.

### Causa raíz (probada por bisección, no conjetural)

La función `aplicarDuty()` del firmware ejecutaba **dos** llamadas a
`ledcWrite()` por invocación: una al canal del modo activo y otra al canal
inactivo para apagarlo «por seguridad». El backend envía un `DUTY` cada Ts; con
Ts=50 ms son ~20 comandos/s, es decir ~40 escrituras LEDC/s alternando entre
dos canales. La API LEDC del núcleo **arduino-esp32 3.x** se bloquea bajo
escrituras muy frecuentes: el `loop()` se colgaba, el ESP32 dejaba de medir,
de transmitir y de leer el serie (NO se reiniciaba; nunca aparecía `READY`
tras el cese).

De ahí los síntomas encadenados: medida congelada (el backend leía la última
línea previa al cuelgue, `V,0.00`), STOP ignorado (el loop no leía serie) y la
falsa «desaceleración tras STOP» (artefacto de buffer del PC, no telemetría
real reanudada).

### Cómo se diagnosticó (cadena de bisecciones)

Se usó un script de prueba mínimo, `test_duty_stream.py` (pyserial síncrono,
sin backend), que envía un flujo de comandos a cadencia configurable e imprime
la telemetría con marca de tiempo. Resultados:

1. Flujo de `DUTY` a 50 ms con pyserial puro → telemetría cesa a ~100 ms.
   Descarta el backend/asyncio/navegador.
2. Mismo fallo con el motor desconectado de OUT1/OUT2 → descarta causa
   eléctrica (brownout, EMI) y dependencia de que el motor gire.
3. `DUTY` a 500 ms → sobrevive 10 s sin fallo. El fallo depende de la
   **frecuencia** de comandos, NO del número total.
4. Token `NOP` (recibido y descartado, sin `ledcWrite`) a 50 ms → NO cuelga.
   Descarta la ruta de recepción/parseo del serie.
5. `DUTY` a 50 ms en modo temperatura → también cuelga. Descarta que sea
   específico del canal del motor; el común es `ledcWrite` en ráfaga.

Conclusión: el único elemento presente en las ramas que fallan y ausente en
`NOP` es `ledcWrite()` invocado a alta frecuencia.

### Corrección aplicada (firmware)

`aplicarDuty()` ahora escribe SOLO el canal del modo activo (una `ledcWrite`
por comando). El apagado del canal inactivo se trasladó a `procesarComando()`,
dentro del bloque `MODE,`, donde se ejecuta una sola vez al cambiar de modo y
no en la ruta caliente de 20 Hz. Validado en la GUI: RPM correctas (~260 a
duty 100, coherente con el monitor serie) y PID estable.

### Limitación honesta / riesgo residual

Está probado que `ledcWrite` en ráfaga era el detonante y que pasar de 2 a 1
escritura por comando lo elimina a Ts=50 ms. NO está demostrado que el margen
sea infinito: con Ts muy por debajo de 50 ms podría reaparecer. Vía de fondo si
ocurre: no reescribir el duty cuando no cambia respecto al anterior, o espaciar
las escrituras al PWM.

## Detalle cosmético pendiente (sin efecto sobre el fallo resuelto)

Línea 44 del firmware: `const uint32_t FREQ_MOT = 1000;` (1 kHz) NO coincide con
su comentario, que dice 10 kHz. 1 kHz cumple el rango del preinforme (>1 kHz) y
el motor funciona. Opcional: corregir el comentario o subir a `10000` para más
margen acústico y mejor promediado de corriente. No tocar sin volver a probar.

## Hardware: estado del montaje

### Subsistema de temperatura: COMPLETO

MOSFET IRLZ44N como low-side switch (Gate por R=1 kΩ a GPIO25, pull-down 10 kΩ).
Resistencia de potencia 10 Ω 5 W. Alimentación: 4 pilas AA en serie (≈6,4 V).
LM35 alimentado del pin 5 V del ESP32, salida a GPIO34. Funciona en lazo
cerrado, pendiente solo de sintonización final del PID.

### Subsistema de velocidad: MONTADO, VERIFICADO Y FUNCIONAL EN LA GUI

- **DRV8833 (módulo clon, chip TI «DRV8833 42K AOEJ»):** los pines `EEP` y `ULT`
  están serigrafiados al revés de su función lógica:
  - `ULT` es en realidad **nSLEEP** (habilitación). DEBE ir a 3,3 V. Flotante =
    chip dormido, OUT1/OUT2 = 0 V.
  - `EEP` es en realidad **nFAULT** (open-drain). NO se conecta (pull-up opcional
    para diagnóstico). Conectarla a 3,3 V como enable fue un error inicial.
  - Serigrafía del módulo: Izq: `IN4, IN3, GND, VCC, IN2, IN1`;
    Der: `EEP, OUT1, OUT2, OUT3, OUT4, ULT`.
  - Pull-down de fábrica de 723 Ω entre `ULT`(nSLEEP) y GND. A 3,3 V el pin del
    ESP32 entrega ≈4,6 mA, aceptable.
- **Cableado del motor:** VCC del DRV8833 → 2 pilas AA dedicadas (≈3,0 V).
  IN1→GPIO26 (PWM), IN2→GPIO27 (dirección, LOW para un sentido). OUT1→cable
  rojo del motor, OUT2→cable blanco. Condensador 100 µF entre VCC y GND.
  IN3/IN4 al aire (canal B sin uso).
- **Encoder N20** (colores del fabricante, NO estándar): Negro→3,3 V del ESP32,
  Azul→GND común, Amarillo→GPIO32 (canal A, interrupción), Verde→GPIO33 (canal
  B, reservado). Verificado que gira y cuenta pulsos correctamente.
- **Alimentaciones:** ESP32 por USB; LM35 del pin 5 V; resistencia calefactora
  de 4 AA (≈6,4 V); motor de 2 AA dedicadas (≈3,0 V); encoder y habilitación del
  DRV8833 del pin 3,3 V del ESP32. Todas las tierras comunes.

### Calibración del encoder: HECHA

`PULSES_PER_REV` calibrado girando el eje de salida 10 vueltas a mano tres veces
(2068, 2069, 2093 pulsos → promedio ≈207 pulsos/vuelta, consistente con encoder
de 7 polos × reducción 1:30). Valor en el firmware: `const float
PULSES_PER_REV = 207.0f;`. `SAMPLE_MS = 50` (telemetría 20 Hz) y
`Serial.setTxBufferSize(2048)` antes de `Serial.begin()`.

## SIGUIENTE TRABAJO (en orden)

1. **Caracterización en lazo abierto del motor** (paso 8 del plan original).
   Ya desbloqueado: la medida de RPM funciona. Caracterizar planta (K, τ, L)
   con escalones de duty en lazo abierto, registrando RPM vs tiempo.
2. **Caracterización en lazo abierto del subsistema térmico** (K, τ, L).
3. **Sintonización del PID** para ambos subsistemas:
   - Ziegler-Nichols por curva de reacción.
   - Térmico: PI (Kd≈0), planta lenta.
   - Motor: medida ruidosa; vigilar amplificación de ruido por Kd (el PID en
     `pid.py` ya tiene filtro derivativo y anti-windup).
   - Dividir Kp entre 2 y refinar a mano con las gráficas P/I/D.
4. Análisis comparativo térmico vs electromecánico (tiempo de respuesta,
   sobreimpulso, error estacionario, estabilidad, efecto de Kp/Ki/Kd).

## Estado de los archivos de código

- `firmware_pid_esp32/firmware_pid_esp32.ino`: CORREGIDO. `aplicarDuty()` con
  una sola `ledcWrite` por comando; apagado del canal inactivo movido al bloque
  `MODE,` de `procesarComando()`. (PULSES_PER_REV=207, SAMPLE_MS=50,
  setTxBufferSize(2048), comando PCOUNT). Pendiente cosmético: comentario de
  FREQ_MOT.
- `pid.py`: sin cambios, se reutiliza. Tiene anti-windup por clamping y filtro
  pasa-bajos en la derivada.
- `server.py`: backend FastAPI. Tarea de lectura con drenaje de backlog
  (conserva solo la última línea), escritura por cola asyncio, control PID a Ts,
  monitor a 10 Hz. NOTA: el «drenaje de backlog» ya no es la solución al
  problema de RPM (ese era el firmware), pero es buena práctica y se conserva.
- `index.html`: frontend Chart.js. Solo emite MODE/RESET/config en acciones de
  usuario, nunca periódicamente.
- `test_duty_stream.py`: script de diagnóstico (pyserial síncrono). Útil para
  futuras pruebas de estrés del enlace serie. Constantes ajustables al inicio:
  PUERTO, BAUDIOS, DUTY_PCT, PERIODO_S, DUR_ACTIVA_S, DUR_POST_S. Recordar que
  el `print` de modo es informativo y puede quedar desincronizado del comando
  MODE real que se envía.
- `gui.py`, `serial_link.py`: OBSOLETOS (referencia histórica).

## Entorno del usuario

- Arch Linux. Usuario `daniel68` (host `Tuxif`), en grupo `uucp` para acceso a
  `/dev/ttyUSB0`.
- Entorno virtual Python: `pid_env`.
- Ruta del proyecto: `~/Development/VirtualInstrumentation/Practica2`.
- Puerto serie del ESP32: `/dev/ttyUSB0` (chip CP2102).
- Compilación/subida del firmware con `arduino-cli`:
  ```
  arduino-cli compile --fqbn esp32:esp32:esp32 firmware_pid_esp32
  arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 firmware_pid_esp32
  ```

## Estilo de respuesta esperado por el usuario

- Lenguaje formal, serio, técnico, sin condescendencia ni emojis.
- Si el usuario se equivoca, decirlo claramente sin suavizarlo. Si el asistente
  se equivoca, reconocerlo con franqueza. NO encadenar conjeturas: exigir
  evidencia que parta el problema en dos mitades disjuntas antes de proponer
  correcciones (esta metodología fue la que resolvió el problema de las RPM).
- Procesos multi-paso: entregar solo el primer paso y avanzar cuando el usuario
  confirme que el anterior funcionó.
- Código siempre con comentarios línea por línea explicando funciones
  desconocidas (el usuario es estudiante y aprende con ellos).
- Particularidades de Arch Linux cuando aplique (PEP 668 / entornos virtuales,
  grupo `uucp`, módulos `cp210x`/`ch341`, AUR para arduino-cli).
- Al final de cada respuesta, una corrección lingüística breve del modo de
  escribir del usuario (principalmente inglés, pero también otros idiomas).
