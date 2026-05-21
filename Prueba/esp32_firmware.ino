#include <Arduino.h>

// --- PIN CONFIGURATION ---
// GPIO25 soporta salida DAC y PWM en ESP32.
// Cambia este pin si usas otro modelo de placa.
#define LED_PIN   25
#define BAUD_RATE 115200

// --- LEDC (PWM) CONFIGURATION ---
// La API ledcAttach() es la forma moderna (ESP32 Arduino Core v3+).
// Frecuencia de 5000 Hz y resolución de 8 bits (valores 0–255).
#define PWM_FREQ       500
#define PWM_RESOLUTION 8      // bits → rango 0–255

// --- STATE ---
bool led_state = false;

// --- FUNCTION DECLARATIONS ---
void processCommand(String command);
void sendResponse(String response);


void setup() {
    Serial.begin(BAUD_RATE);

    // Configura el pin como salida PWM con LEDC
    ledcAttach(LED_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(LED_PIN, 0);   // Empieza apagado

    // Espera a que el puerto serial esté listo
    while (!Serial) {
        delay(10);
    }

    sendResponse("ESP32_READY");
}


void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();   // Elimina \r, \n y espacios extra

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

    // --- CONTROL DIGITAL DEL LED (on/off) ---
    else if (command == "ENCENDER_LED") {
        ledcWrite(LED_PIN, 255);
        led_state = true;
        sendResponse("LED_ON");
    }

    else if (command == "APAGAR_LED") {
        ledcWrite(LED_PIN, 0);
        led_state = false;
        sendResponse("LED_OFF");
    }

    else if (command == "TOGGLE_LED") {
        led_state = !led_state;
        ledcWrite(LED_PIN, led_state ? 255 : 0);
        sendResponse(led_state ? "LED_ON" : "LED_OFF");
    }

    // --- ESTADO ---
    else if (command == "STATUS_LED") {
        sendResponse(led_state ? "LED_IS_ON" : "LED_IS_OFF");
    }

    // --- CONTROL PWM ANALÓGICO ---
    // Recibe un entero 0–255 desde Python, p. ej. "SET_PWM:127"
    else if (command.startsWith("SET_PWM:")) {
        int val = command.substring(8).toInt();
        val = constrain(val, 0, 255);   // Protege el rango
        ledcWrite(LED_PIN, val);
        // No se envía respuesta en este modo para no bloquear
        // la cadencia de la onda. Descomenta si necesitas debug:
        // sendResponse("PWM_SET:" + String(val));
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
