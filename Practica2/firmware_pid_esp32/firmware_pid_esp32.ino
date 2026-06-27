/*
  ============================================================================
  Practica 2 - Instrumentacion Virtual
  Firmware del ESP32: capa de hardware (I/O) para el control PID
  de temperatura y de velocidad.
  ============================================================================

  El controlador PID se ejecuta en el PC (Python). Este firmware NO calcula
  el PID; unicamente:
    - Genera la senal PWM (temperatura: 15 Hz; motor: 10 kHz).
    - Lee el sensor LM35 por ADC y lo convierte a grados Celsius.
    - Cuenta los flancos del encoder (canal A) y calcula las RPM.
    - Se comunica con el PC por el puerto serie (USB, UART0) mediante un
      protocolo de texto ASCII terminado en salto de linea ('\n').

  Nucleo objetivo: arduino-esp32 version 3.x
    API PWM nueva:  ledcAttach(pin, frecuencia, resolucion)
                    ledcWrite(pin, cuentas)
    (En el nucleo 2.x se usaba ledcSetup + ledcAttachPin + ledcWrite(canal,...).
     Si compilas con el nucleo 2.x debes adaptar esas tres llamadas.)
  ============================================================================
*/

// ---------------------------------------------------------------------------
//  Asignacion de pines (segun figuras 3 y 5 del preinforme)
// ---------------------------------------------------------------------------
const int PIN_LM35     = 34;  // Entrada ADC del LM35 (GPIO34: solo entrada, sin pull interno)
const int PIN_PWM_TEMP = 25;  // Salida PWM hacia la base del TIP122 (a traves de R de 1 kOhm)
const int PIN_PWM_MOT  = 26;  // Salida PWM hacia IN1 del DRV8833
const int PIN_DIR_MOT  = 27;  // Salida de direccion hacia IN2 del DRV8833
const int PIN_ENC_A    = 32;  // Canal A del encoder (entrada con interrupcion)
const int PIN_ENC_B    = 33;  // Canal B del encoder (no se usa en el lazo; opcional, para sentido)

// Nota: solo usamos Serial (UART0 sobre USB) para hablar con el PC. En el
// nucleo 3.x los pines por defecto de UART1/UART2 cambiaron a GPIO26/27/25,
// pero eso solo afecta si se instancia Serial1/Serial2, cosa que NO hacemos.

// ---------------------------------------------------------------------------
//  Parametros de la PWM
// ---------------------------------------------------------------------------
const int      PWM_RES_BITS = 10;                      // Resolucion de la PWM en bits
const uint32_t PWM_MAX      = (1 << PWM_RES_BITS) - 1; // Cuentas maximas: 2^10 - 1 = 1023
const uint32_t FREQ_TEMP    = 15;                      // 15 Hz para la planta termica (rango 10-20 Hz)
const uint32_t FREQ_MOT     = 1000;                   // 10 kHz para el motor (rango 5-20 kHz)

// ---------------------------------------------------------------------------
//  Parametros del encoder
// ---------------------------------------------------------------------------
// Flancos de SUBIDA del canal A por cada vuelta COMPLETA del eje de SALIDA.
// Incluye la relacion de reduccion de la caja. ESTE VALOR DEBE CALIBRARSE
// para tu motor concreto (ver explicacion en el mensaje). El valor de abajo
// es solo un punto de partida tipico para un N20 de 250 RPM.
const float PULSES_PER_REV = 207.0f;

// ---------------------------------------------------------------------------
//  Temporizacion de muestreo / telemetria
// ---------------------------------------------------------------------------
const uint32_t SAMPLE_MS = 50;   // Cada 20 ms se mide y se envia telemetria (50 Hz)

// ---------------------------------------------------------------------------
//  Estado de operacion
// ---------------------------------------------------------------------------
enum Modo { MODO_TEMP, MODO_VEL };  // Tipo enumerado con los dos subsistemas
Modo modoActual = MODO_TEMP;        // Modo por defecto al arrancar

// ---------------------------------------------------------------------------
//  Variables compartidas con la rutina de interrupcion (ISR)
// ---------------------------------------------------------------------------
// 'volatile' obliga al compilador a leer la variable desde memoria cada vez,
// evitando optimizaciones incorrectas en variables que cambia una interrupcion.
volatile uint32_t contadorPulsos = 0;

// ---------------------------------------------------------------------------
//  Variables de control y temporizacion
// ---------------------------------------------------------------------------
float    dutyActual     = 0.0f;  // Ciclo de trabajo aplicado actualmente [0..100]
uint32_t tUltimaMuestra = 0;     // Marca de tiempo de la ultima muestra (millis)

// Buffer de recepcion: acumula caracteres hasta recibir un '\n'
String lineaRx = "";

// ============================================================================
//  ISR del encoder
//  Se ejecuta automaticamente en cada flanco de subida del canal A.
//  IRAM_ATTR coloca la funcion en la RAM interna (IRAM): es obligatorio en el
//  ESP32 para que la ISR sea rapida y segura. Dentro de una ISR solo se hace
//  trabajo minimo; aqui, incrementar el contador.
// ============================================================================
void IRAM_ATTR isrEncoder() {
  contadorPulsos++;
}

// ============================================================================
//  Aplica el ciclo de trabajo (0..100 %) al pin PWM del subsistema activo
//  y apaga el subsistema inactivo por seguridad.
// ============================================================================
void aplicarDuty(float dutyPct) {
  // Saturacion de seguridad: el duty siempre queda dentro de [0, 100]
  if (dutyPct < 0.0f)   dutyPct = 0.0f;
  if (dutyPct > 100.0f) dutyPct = 100.0f;
  dutyActual = dutyPct;

  // Convierte el porcentaje (0..100) a cuentas de PWM (0..PWM_MAX).
  // El + 0.5 sirve para redondear al entero mas cercano al truncar.
  uint32_t cuentas = (uint32_t)((dutyPct / 100.0f) * PWM_MAX + 0.5f);

  if (modoActual == MODO_TEMP) {
    ledcWrite(PIN_PWM_TEMP, cuentas);  // PWM a la base del TIP122
    ledcWrite(PIN_PWM_MOT, 0);         // Aseguramos el motor apagado
  } else {
    ledcWrite(PIN_PWM_MOT, cuentas);   // PWM a IN1 del DRV8833
    ledcWrite(PIN_PWM_TEMP, 0);        // Aseguramos la resistencia apagada
  }
}

// ============================================================================
//  Lee el LM35 promediando varias muestras del ADC y devuelve grados Celsius.
//  analogReadMilliVolts() lee el ADC en crudo y le aplica la calibracion de
//  fabrica (eFuse) del ESP32, devolviendo milivoltios ya corregidos. Asi no
//  hay que convertir cuentas a voltaje a mano ni preocuparse por la Vref.
//  La atenuacion por defecto cubre un rango aprox. de 0 a 3.1 V, mas que
//  suficiente para el LM35 (300-350 mV en el rango 30-35 C).
// ============================================================================
float leerTemperaturaC() {
  const int N = 16;            // Numero de muestras a promediar (filtro simple)
  uint32_t acumulado = 0;
  for (int i = 0; i < N; i++) {
    acumulado += analogReadMilliVolts(PIN_LM35);  // Lectura calibrada en mV
  }
  float mV = (float)acumulado / N;  // Promedio en milivoltios
  return mV / 10.0f;                // LM35: 10 mV por C  ->  T[C] = mV / 10
}

// ============================================================================
//  Calcula las RPM del eje de salida a partir de los pulsos acumulados
//  durante la ventana de tiempo transcurrida dt_ms.
//  Aplica la ecuacion: RPM = (pulsos / pulsos_por_vuelta) * (60 / dt[s]).
// ============================================================================
float calcularRPM(uint32_t dt_ms) {
  // Lectura "atomica" del contador: deshabilitamos las interrupciones un
  // instante para leer y reiniciar el contador sin que la ISR lo modifique
  // en mitad de la operacion.
  noInterrupts();
  uint32_t pulsos = contadorPulsos;
  contadorPulsos = 0;             // Reinicia para la siguiente ventana
  interrupts();

  if (dt_ms == 0) return 0.0f;    // Evita la division por cero

  float vueltas = (float)pulsos / PULSES_PER_REV;
  // 60 s expresados en ms son 60000; dt_ms ya esta en ms.
  float rpm = vueltas * (60000.0f / (float)dt_ms);
  return rpm;
}

// ============================================================================
//  Procesa una linea de comando recibida desde el PC.
//  Protocolo (texto ASCII, terminado en '\n'):
//    MODE,T    -> selecciona el modo temperatura
//    MODE,V    -> selecciona el modo velocidad (motor)
//    DUTY,xx   -> fija el ciclo de trabajo (xx = numero entre 0 y 100)
//    STOP      -> pone duty = 0 (no cambia de modo)
//    RESET     -> pone duty = 0 y reinicia el contador del encoder
//    PING      -> responde "PONG" (sirve para verificar la conexion)
// ============================================================================
void procesarComando(String cmd) {
  cmd.trim();  // Elimina espacios y saltos de linea sobrantes en los extremos

  if (cmd.startsWith("MODE,")) {
    char m = cmd.charAt(5);          // Caracter inmediatamente despues de "MODE,"
    if (m == 'T') {
      modoActual = MODO_TEMP;
      aplicarDuty(0.0f);             // Al cambiar de modo, apagamos por seguridad
    } else if (m == 'V') {
      modoActual = MODO_VEL;
      aplicarDuty(0.0f);
      noInterrupts();
      contadorPulsos = 0;            // Empezamos la medida de RPM desde cero
      interrupts();
    }
  }
  else if (cmd.startsWith("DUTY,")) {
    float d = cmd.substring(5).toFloat();  // Texto tras la coma -> numero flotante
    aplicarDuty(d);
  }
  else if (cmd == "STOP") {
    aplicarDuty(0.0f);
  }
  else if (cmd == "RESET") {
    aplicarDuty(0.0f);
    noInterrupts();
    contadorPulsos = 0;
    interrupts();
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
  }
    else if (cmd == "PCOUNT") {
    // Lectura atomica: deshabilitamos interrupciones para leer el contador
    // sin que la ISR lo modifique en mitad de la operacion.
    noInterrupts();
    uint32_t n = contadorPulsos;   // Copia local del valor protegido
    interrupts();
    Serial.print("PCOUNT,");       // Prefijo identificador del mensaje
    Serial.println(n);             // Imprime el numero y termina con '\n'
  }
}

// ============================================================================
//  Configuracion inicial (se ejecuta una sola vez al arrancar)
// ============================================================================
void setup() {
  // Amplia el buffer de transmision del UART para evitar bloqueos por
  // backpressure si el PC tarda momentaneamente en drenar la telemetria.
  // Por defecto son 256 bytes; 2048 bytes dan margen para ~120 lineas.
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);             // UART0 sobre USB hacia el PC, 115200 baudios

  // --- Pines de salida digital ---
  pinMode(PIN_DIR_MOT, OUTPUT);
  digitalWrite(PIN_DIR_MOT, LOW);   // IN2 = 0  ->  giro en un solo sentido (avance)

  // --- PWM (API del nucleo 3.x) ---
  // ledcAttach(pin, frecuencia, resolucion) configura el pin como salida PWM
  // y le asigna automaticamente un canal y un temporizador internos.
  ledcAttach(PIN_PWM_TEMP, FREQ_TEMP, PWM_RES_BITS);  // PWM de la resistencia
  ledcAttach(PIN_PWM_MOT,  FREQ_MOT,  PWM_RES_BITS);  // PWM del motor
  ledcWrite(PIN_PWM_TEMP, 0);       // Arrancamos con duty 0 en ambos
  ledcWrite(PIN_PWM_MOT,  0);

  // --- Encoder ---
  pinMode(PIN_ENC_A, INPUT_PULLUP); // Canal A con resistencia pull-up interna
  pinMode(PIN_ENC_B, INPUT_PULLUP); // Canal B (reservado; no se usa en el lazo)
  // attachInterrupt asocia la funcion isrEncoder al flanco de SUBIDA del canal A.
  // digitalPinToInterrupt traduce el numero de pin al numero de interrupcion.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, RISING);

  tUltimaMuestra = millis();        // millis() devuelve los ms desde el arranque

  Serial.println("READY");          // Avisa al PC de que el firmware esta listo
}

// ============================================================================
//  Bucle principal (se repite indefinidamente)
// ============================================================================
void loop() {
  // ----- 1) Lectura no bloqueante de comandos entrantes -----
  while (Serial.available() > 0) {       // Mientras haya bytes en el buffer de RX
    char c = (char)Serial.read();        // Lee un byte
    if (c == '\n') {                     // Fin de linea: procesamos el comando
      procesarComando(lineaRx);
      lineaRx = "";                      // Vaciamos el buffer para el siguiente
    } else if (c != '\r') {              // Ignoramos el retorno de carro
      lineaRx += c;                      // Acumulamos el caracter
    }
  }

  // ----- 2) Medicion periodica y envio de telemetria -----
  uint32_t ahora = millis();
  if (ahora - tUltimaMuestra >= SAMPLE_MS) {
    uint32_t dt = ahora - tUltimaMuestra;  // Ventana real transcurrida (ms)
    tUltimaMuestra = ahora;

    if (modoActual == MODO_TEMP) {
      float tempC = leerTemperaturaC();
      // Formato de telemetria:  T,<temperatura>,<duty>
      Serial.print("T,");
      Serial.print(tempC, 2);            // 2 decimales
      Serial.print(",");
      Serial.println(dutyActual, 2);
    } else {
      float rpm = calcularRPM(dt);
      // Formato de telemetria:  V,<rpm>,<duty>
      Serial.print("V,");
      Serial.print(rpm, 2);
      Serial.print(",");
      Serial.println(dutyActual, 2);
    }
  }
}
