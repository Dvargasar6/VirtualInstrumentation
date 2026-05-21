#include <Arduino.h>

// --- PIN CONFIGURATION ---
#define LED_PIN 2        // Built-in LED on most ESP32 boards
#define BAUD_RATE 115200
#define SERIAL_TIMEOUT 100  // ms to wait for complete command

// --- STATE ---
bool led_state = false;

// --- FUNCTION DECLARATIONS ---
void processCommand(String command);
void sendResponse(String response);


void setup() {
    Serial.begin(BAUD_RATE);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Wait for serial port to be ready
    while (!Serial) {
        delay(10);
    }

    sendResponse("ESP32_READY");
}


void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove \r\n and whitespace

        if (command.length() > 0) {
            processCommand(command);
        }
    }
}


void processCommand(String command) {

    // --- CONNECTION VERIFICATION ---
    if (command == "PING") {
        sendResponse("PONG");
    }

    // --- LED CONTROL ---
    else if (command == "ENCENDER_LED") {
        digitalWrite(LED_PIN, HIGH);
        led_state = true;
        sendResponse("LED_ON");
    }


    else if (command == "APAGAR_LED") {
        digitalWrite(LED_PIN, LOW);
        led_state = false;
        sendResponse("LED_OFF");
    }

    else if (command == "TOGGLE_LED") {
        led_state = !led_state;
        digitalWrite(LED_PIN, led_state ? HIGH : LOW);
        sendResponse(led_state ? "LED_ON" : "LED_OFF");
    }

    // --- STATUS ---
    else if (command == "STATUS_LED") {
        sendResponse(led_state ? "LED_IS_ON" : "LED_IS_OFF");
    }
    else if (command.startsWith("SET_PWM:")) {
        int val = command.substring(8).toInt();
        val = constrain(val, 0, 255);
        ledcWrite(LED_PIN, val);
        sendResponse("PWM_SET:" + String(val));
    }

    else if (command == "STATUS") {
        sendResponse("ESP32_OK");
    }

    // --- UNKNOWN COMMAND ---
    else {
        sendResponse("ERROR_UNKNOWN_CMD:" + command);
    }
}


void sendResponse(String response) {
    Serial.println(response);
}