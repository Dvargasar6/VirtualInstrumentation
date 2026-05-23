#include <Arduino.h>

// --- PIN CONFIGURATION ---
// GPIO25 tiene DAC de 8 bits nativo → entrega voltaje analógico real 0–3.3 V.
// Conectar al canal 1 del osciloscopio (sonda) y GND del ESP32 al GND de la sonda.
// NO usar ledcAttach ni ledcWrite en este pin cuando se usa el DAC.
#define DAC_PIN   25      // Solo GPIO25 o GPIO26 soportan DAC en ESP32
#define ADC_PIN   34      // GPIO34 es solo entrada, ideal para leer voltajes (0-3.3 V) sin riesgo de daño.
#define BAUD_RATE 230400

// LED auxiliar en GPIO2 para indicación visual (no DAC, usa digitalWrite)
#define LED_AUX   2

// --- STATE ---
bool led_state = false;
// control de streaming ADC
bool streamingADC = false;

// --- FUNCTION DECLARATIONS ---
void processCommand(String command);
void sendResponse(String response);


void setup() {
    Serial.begin(BAUD_RATE);

    // El DAC no requiere pinMode ni ledcAttach.
    // dacWrite inicializa el pin automáticamente.
    dacWrite(DAC_PIN, 0);   // Inicia en 0 V

    pinMode(LED_AUX, OUTPUT);
    digitalWrite(LED_AUX, LOW);
    //Configuración del ADC para lectura de voltaje
    pinMode(ADC_PIN, INPUT);

    while (!Serial) {
        delay(10);
    }

    sendResponse("ESP32_READY");
    
}

int contador_envio = 0;

void loop() {
    // ADC
    if (streamingADC) {
        //Lectura ADC RAW (0-4095) y envío por Serial
        int adc_value = analogRead(ADC_PIN);
        float voltaje = adc_value * (3.3 / 4095.0);
        // SUMAREMOS EN EL CONTADOR
        contador_envio++;
        //Enviar solo 1 se cada 10 muestras
        if (contador_envio >= 10) {
            Serial.println(voltaje);
            contador_envio = 0;
        }
        delayMicroseconds(200); // 20 kHz aprox
    }
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() > 0) {
            processCommand(command);
        }
    
    }

}


void processCommand(String command) {

    // --- VERIFICACIÓN DE CONEXIÓN ---
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

    // --- ESTADO ---
    else if (command == "STATUS_LED") {
        sendResponse(led_state ? "LED_IS_ON" : "LED_IS_OFF");
    }

    // --- SALIDA DAC ANALOGICA ---
    // Recibe un entero 0-255 desde Python -> genera voltaje 0-3.3 V en GPIO25.
    // Este es el comando principal para la señal AM modulada.
    else if (command.startsWith("SET_PWM:")) {
        int val = command.substring(8).toInt();
        val = constrain(val, 0, 255);
        dacWrite(DAC_PIN, val);   // Voltaje real: val/255 * 3.3 V
    }

    else if (command == "START_ADC") {
        streamingADC = true;
        sendResponse("ADC_STREAMING_ON");
    }
    // Detener streaming ADC
    else if (command == "STOP_ADC") {
        streamingADC = false;
        sendResponse("ADC_STREAMING_OFF");
    }

    // --- APAGAR DAC ---
    else if (command == "DAC_OFF") {
        dacWrite(DAC_PIN, 0);
        sendResponse("DAC_OFF");
    }

    else if (command == "STATUS") {
        sendResponse("ESP32_OK");
    }


    // --- COMANDO DESCONOCIDO ---
    else {
        sendResponse("ERROR_UNKNOWN_CMD:" + command);
    }
}


void sendResponse(String response) {
    Serial.println(response);
}
