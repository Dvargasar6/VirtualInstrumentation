#include <Arduino.h>

#define LED_PIN    4        // GPIO4 — no boot conflicts
#define BAUD_RATE  115200
#define PWM_FREQ   5000
#define PWM_RES    8        // 0-255

bool sine_running = false;
float sine_angle  = 0.0;
unsigned long last_update = 0;
const int UPDATE_INTERVAL_MS = 20;

void sendResponse(String response) {
    Serial.println(response);
}

void processCommand(String command) {
    if (command == "PING") {
        sendResponse("PONG");
    }
    else if (command.startsWith("SET_PWM:")) {
        int val = command.substring(8).toInt();
        val = constrain(val, 0, 255);
        ledcWrite(LED_PIN, val);
        sendResponse("PWM_SET:" + String(val));
    }
    else if (command == "STATUS") {
        sendResponse("SINE_IDLE");
    }
    else {
        sendResponse("ERROR_UNKNOWN_CMD:" + command);
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    ledcAttach(LED_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(LED_PIN, 0);
    while (!Serial) { delay(10); }
    sendResponse("ESP32_READY");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.length() > 0) {
            processCommand(command);
        }
    }
}