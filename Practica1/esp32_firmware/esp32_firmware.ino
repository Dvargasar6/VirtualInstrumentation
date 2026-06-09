#include <Arduino.h>
#include <math.h>

// --- PIN CONFIGURATION ---
// GPIO25 tiene DAC de 8 bits nativo -> entrega voltaje analogico real 0-3.3 V.
// NO usar ledcAttach ni ledcWrite en este pin cuando se usa el DAC.
#define DAC_PIN   25      // Solo GPIO25 o GPIO26 soportan DAC en ESP32
#define ADC_PIN   34      // GPIO34 es solo entrada
#define BAUD_RATE 230400

// LED auxiliar (no DAC)
#define LED_AUX   2

// --- ESTADO ---
bool led_state    = false;
bool streamingADC = false;   // envia muestras ADC por serial
bool streamingAM  = false;   // genera senal AM en el DAC

// --- PARAMETROS AM ---
// Volatile porque se modifican desde processCommand y se leen desde loop;
// con esto el compilador no asume que el valor no cambia entre lecturas.
// (En la practica el ESP32 es single-core efectivo en este Arduino core, pero
// volatile mantiene la semantica correcta y previene optimizaciones erroneas.)
volatile float am_Ac     = 60.0f;   // amplitud portadora (escala DAC 0-255)
volatile float am_Am     = 60.0f;   // amplitud moduladora
volatile float am_fc     = 80.0f;   // frecuencia portadora [Hz]
volatile float am_fm     = 10.0f;   // frecuencia moduladora [Hz]
volatile float am_offset = 120.0f;  // nivel DC (escala DAC 0-255)

// Tiempo de referencia para la generacion AM. Se resetea al iniciar streamingAM
// para que la senal arranque siempre desde fase 0.
unsigned long am_t0_us = 0;

// --- DECLARACIONES ---
void processCommand(String command);
void sendResponse(String response);
void updateAM();


void setup() {
    Serial.begin(BAUD_RATE);

    // dacWrite inicializa el pin DAC automaticamente
    dacWrite(DAC_PIN, 0);

    pinMode(LED_AUX, OUTPUT);
    digitalWrite(LED_AUX, LOW);
    pinMode(ADC_PIN, INPUT);

    while (!Serial) {
        delay(10);
    }

    sendResponse("ESP32_READY");
}


void loop() {
    // --- GENERACION AM ---
    // Se ejecuta lo mas rapido posible (sin delay) para maximizar la tasa de
    // actualizacion del DAC. Cada iteracion de updateAM toma ~3-5 us con la
    // FPU del ESP32, dando una tasa real de ~200-300 kHz cuando no hay otra
    // carga, o ~30-50 kHz si se intercala con ADC streaming y comandos.
    if (streamingAM) {
        updateAM();
    }

    // --- ADC STREAMING ---
    // Se ejecuta condicionalmente con un periodo objetivo de 500 us (~2 kHz).
    // Usamos una variable estatica para acumular tiempo y solo leer cuando
    // toca, en vez de delayMicroseconds que bloquearia la generacion AM.
    static unsigned long ultimo_adc_us = 0;
    if (streamingADC) {
        unsigned long ahora = micros();
        if (ahora - ultimo_adc_us >= 500) {
            ultimo_adc_us = ahora;
            int adc_value = analogRead(ADC_PIN);
            Serial.println(adc_value);
        }
    }

    // --- COMANDOS ---
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() > 0) {
            processCommand(command);
        }
    }
}


// --- GENERACION DE UN PUNTO DE LA SENAL AM ---
// Se llama en cada iteracion del loop cuando streamingAM esta activo.
// Calcula s(t) = offset + Ac*(1 + (Am/Ac)*sin(2*pi*fm*t)) * sin(2*pi*fc*t)
// y lo escribe al DAC. Usa micros() para tiempo real, garantizando que las
// frecuencias fc y fm sean exactas independientemente del tiempo que tome el loop.
void updateAM() {
    // t en segundos desde que se inicio el streaming AM
    float t = (micros() - am_t0_us) * 1.0e-6f;

    // Productos sinusoidales. sinf() es la version float (mas rapida que sin()
    // que opera en double). El ESP32 tiene FPU hardware para float.
    // 2*pi precalculado como constante.
    const float TWO_PI_F = 6.2831853f;
    float moduladora = sinf(TWO_PI_F * am_fm * t);
    float portadora  = sinf(TWO_PI_F * am_fc * t);

    // Formula AM convencional
    float s = am_offset + am_Ac * (1.0f + (am_Am / am_Ac) * moduladora) * portadora;

    // Clip al rango del DAC y conversion a entero
    int valor = (int)s;
    if (valor < 0)   valor = 0;
    if (valor > 255) valor = 255;

    dacWrite(DAC_PIN, valor);
}


void processCommand(String command) {

    // --- VERIFICACION DE CONEXION ---
    if (command == "PING") {
        sendResponse("PONG");
    }

    // --- CONTROL DIGITAL DEL LED AUXILIAR ---
    else if (command == "ENCENDER_LED") {
        digitalWrite(LED_AUX, HIGH);
        led_state = true;
        sendResponse("LED_ON");
    }
    else if (command == "APAGAR_LED") {
        digitalWrite(LED_AUX, LOW);
        led_state = false;
        sendResponse("LED_OFF");
    }
    else if (command == "TOGGLE_LED") {
        led_state = !led_state;
        digitalWrite(LED_AUX, led_state ? HIGH : LOW);
        sendResponse(led_state ? "LED_ON" : "LED_OFF");
    }
    else if (command == "STATUS_LED") {
        sendResponse(led_state ? "LED_IS_ON" : "LED_IS_OFF");
    }

    // --- DAC MANUAL (compatibilidad: aun se acepta SET_PWM para debug) ---
    else if (command.startsWith("SET_PWM:")) {
        // Solo tiene efecto si NO esta corriendo el streaming AM. Si esta
        // corriendo, el siguiente updateAM() sobrescribira el valor.
        int val = command.substring(8).toInt();
        val = constrain(val, 0, 255);
        dacWrite(DAC_PIN, val);
    }

    // --- ADC STREAMING ---
    else if (command == "START_ADC") {
        streamingADC = true;
        sendResponse("ADC_STREAMING_ON");
    }
    else if (command == "STOP_ADC") {
        streamingADC = false;
        sendResponse("ADC_STREAMING_OFF");
    }

    // --- AM STREAMING ---
    // Inicia la generacion de la senal AM en el DAC.
    // Resetea la referencia de tiempo para que la senal arranque en fase 0.
    else if (command == "START_AM") {
        am_t0_us = micros();
        streamingAM = true;
        sendResponse("AM_STREAMING_ON");
    }
    else if (command == "STOP_AM") {
        streamingAM = false;
        dacWrite(DAC_PIN, (int)am_offset);   // dejar el DAC en el offset DC
        sendResponse("AM_STREAMING_OFF");
    }

    // --- ACTUALIZAR PARAMETROS AM ---
    // Formato: SET_AM:Ac,Am,fc,fm,offset
    // Ejemplo: SET_AM:60,60,80,10,120
    // Los 5 valores se aplican atomicamente: aunque updateAM se ejecute en
    // medio del cambio, no veria una mezcla parcial porque cada asignacion
    // individual de float en ESP32 es atomica (4 bytes, alineados).
    else if (command.startsWith("SET_AM:")) {
        String params = command.substring(7);
        // Parseo manual de los 5 valores separados por coma
        int idx[4];   // posiciones de las 4 comas
        int n = 0;
        for (int i = 0; i < (int)params.length() && n < 4; i++) {
            if (params.charAt(i) == ',') {
                idx[n++] = i;
            }
        }
        if (n == 4) {
            am_Ac     = params.substring(0, idx[0]).toFloat();
            am_Am     = params.substring(idx[0]+1, idx[1]).toFloat();
            am_fc     = params.substring(idx[1]+1, idx[2]).toFloat();
            am_fm     = params.substring(idx[2]+1, idx[3]).toFloat();
            am_offset = params.substring(idx[3]+1).toFloat();
            sendResponse("AM_PARAMS_SET");
        } else {
            sendResponse("ERROR_BAD_AM_FORMAT");
        }
    }

    // --- CONSULTAR PARAMETROS AM ACTUALES ---
    // Devuelve "AM_PARAMS:Ac,Am,fc,fm,offset" para que Python sincronice
    // sus sliders con el estado real del ESP32 al conectarse.
    else if (command == "GET_AM") {
        String resp = "AM_PARAMS:";
        resp += String(am_Ac, 2)     + ",";
        resp += String(am_Am, 2)     + ",";
        resp += String(am_fc, 2)     + ",";
        resp += String(am_fm, 2)     + ",";
        resp += String(am_offset, 2);
        sendResponse(resp);
    }

    // --- APAGAR DAC ---
    else if (command == "DAC_OFF") {
        streamingAM = false;
        dacWrite(DAC_PIN, 0);
        sendResponse("DAC_OFF");
    }

    else if (command == "STATUS") {
        sendResponse("ESP32_OK");
    }

    else {
        sendResponse("ERROR_UNKNOWN_CMD:" + command);
    }
}


void sendResponse(String response) {
    Serial.println(response);
}
