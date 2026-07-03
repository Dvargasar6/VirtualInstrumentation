// ============================================================================
//  Parametros globales:
// ============================================================================


// Asignacion de pines:
const int PIN_LM35     = 34;  // Entrada ADC del LM35
const int PIN_PWM_TEMP = 25;  // Salida PWM hacia la base del TIP122
const int PIN_PWM_MOT  = 26;  // Salida PWM hacia IN1 del DRV8833
const int PIN_DIR_MOT  = 27;  // Salida de direccion hacia IN2 del DRV8833
const int PIN_ENC_A    = 32;  // Canal A del encoder
const int PIN_ENC_B    = 33;  // Canal B del encoder 

// Parametros del PWM:
const int PWM_RES_BITS = 10;                      // Resolucion de la PWM en bits
const uint32_t PWM_MAX = (1 << PWM_RES_BITS) - 1; // Cuentas maximas: 2^10 - 1 = 1023
const uint32_t FREQ_TEMP = 15;                    // Temperatura (rango 10-20 Hz)
const uint32_t FREQ_MOT = 10000;                   // Motor (rango 5-20 kHz)


const float PULSES_PER_REV = 207.0f;   // Pulsos por revolucion del encoder
const uint32_t SAMPLE_MS = 50;         // Muestras por ms

// Variables globales:
volatile uint32_t contadorPulsos = 0;  // Contador de la ISR
float dutyActual = 0.0f;               // Ciclo de trabajo
uint32_t tUltimaMuestra = 0;           // Marca de tiempo (ms)
String lineaRx = "";                   // Buffer de recepcion


enum Modo { MODO_TEMP, MODO_VEL };  // Subsistemas (Temperatura, Velocidad)
Modo modoActual = MODO_TEMP;        // Modo Temperatura por defecto

// ============================================================================
//  Funciones globales:
// ============================================================================

// ISR del encoder:
void IRAM_ATTR isrEncoder() {
  contadorPulsos++;
}


// Funcion de configuracion del duty cycle al sistema activo:
void aplicarDuty(float dutyPct) {
  // Saturacion de seguridad: el duty siempre queda dentro de [0, 100]
  if (dutyPct < 0.0f)   dutyPct = 0.0f;
  if (dutyPct > 100.0f) dutyPct = 100.0f;
  dutyActual = dutyPct;

  // Convierte el porcentaje (0..100) a cuentas de PWM (0..PWM_MAX).
  uint32_t cuentas = (uint32_t)((dutyPct / 100.0f) * PWM_MAX + 0.5f);

  // Se enciende el pin PWM del subsistema activo.
  if (modoActual == MODO_TEMP) {
    ledcWrite(PIN_PWM_TEMP, cuentas);  // PWM a gate del IRLZ44N
  } else {
    ledcWrite(PIN_PWM_MOT, cuentas);   // PWM a IN1 del DRV8833
  }
}


// Funcion de ADC para obtener el valor de la temperatura:
float leerTemperaturaC() {
  const int N = 16;            // Numero de muestras a promediar
  uint32_t acumulado = 0;
  for (int i = 0; i < N; i++) {
    acumulado += analogReadMilliVolts(PIN_LM35);  // Lectura calibrada en mV
  }
  float mV = (float)acumulado / N;  // Promedio en milivoltios
  return mV / 10.0f;                // LM35: 10 mV por C  ->  T[C] = mV / 10
}


//  Funcion que calcula las RPM del motor.
//  Aplica la ecuacion: RPM = (pulsos / pulsos_por_vuelta) * (60 / dt[s]):
float calcularRPM(uint32_t dt_ms) {

  noInterrupts();
  uint32_t pulsos = contadorPulsos;
  contadorPulsos = 0;             
  interrupts();

  if (dt_ms == 0) return 0.0f;    

  float vueltas = (float)pulsos / PULSES_PER_REV;
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
  cmd.trim();

  if (cmd.startsWith("MODE,")) {
    char m = cmd.charAt(5);
    if (m == 'T') {
      modoActual = MODO_TEMP;
      ledcWrite(PIN_PWM_MOT, 0);     // Apagamos el canal del motor
      aplicarDuty(0.0f);             // Ponemos el canal termico a 0
    } else if (m == 'V') {
      modoActual = MODO_VEL;
      ledcWrite(PIN_PWM_TEMP, 0);    // Apagamos el canal termico
      aplicarDuty(0.0f);             // Ponemos el canal del motor a 0
      noInterrupts();
      contadorPulsos = 0;            // Empezamos la medida de RPM desde cero
      interrupts();
    }
    Serial.print("MODE,");
    Serial.println(m);
  }
  else if (cmd.startsWith("DUTY,")) {
    float d = cmd.substring(5).toFloat();  // Duty cycle
    aplicarDuty(d);
    Serial.print("DUTY,");
    Serial.println(d);
  }
  else if (cmd == "STOP") {
    aplicarDuty(0.0f);
    Serial.println("STOP");
  }
  else if (cmd == "RESET") {
    aplicarDuty(0.0f);
    noInterrupts();
    contadorPulsos = 0;
    interrupts();
    Serial.println("RESET");
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
  }
  else if (cmd == "PCOUNT") {
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

  // Configuraciones de los pines PWM:
  ledcAttach(PIN_PWM_TEMP, FREQ_TEMP, PWM_RES_BITS);  // PWM de la resistencia
  ledcAttach(PIN_PWM_MOT,  FREQ_MOT,  PWM_RES_BITS);  // PWM del motor
  ledcWrite(PIN_PWM_TEMP, 0);
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
