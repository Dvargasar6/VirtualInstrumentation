// Practica 3 - Virtual Instrumentation
// ESP32 firmware: reads LM35, Hall 3144 and KY-036, drives an RGB LED by PWM,
// and exchanges JSON lines over USB serial with the Python server.
//
// Serial protocol (115200 baud, one JSON object per line):
//   ESP32 -> PC (every 100 ms): {"t":24.5,"vh":3.28,"ky":0,"vky":1.20}
//     t   : temperature in Celsius from the LM35
//     vh  : voltage on the Hall 3144 output line (V)
//     ky  : digital state of the KY-036 touch output (0/1)
//     vky : analog voltage of the KY-036 (V)
//   PC -> ESP32: {"cmd":"rgb","r":0,"g":255,"b":0}  -> set LED color
//                {"cmd":"off"}                      -> LED off
//
// The control state machine lives in the Python server; this firmware is a
// deterministic I/O bridge, which keeps the instrument logic on the PC side.

#include <Arduino.h>
#include <ArduinoJson.h>   // JSON parsing/serialization (installed via lib_deps)

// ---------- Pin map (must match context.md) ----------
const int PIN_LM35  = 34;  // ADC1_CH6, input-only: LM35 VOUT (10 mV/C)
const int PIN_HALL  = 35;  // ADC1_CH7, input-only: 3144 OUT with 10k pull-up to 3V3
const int PIN_KY_AO = 32;  // ADC1_CH4: KY-036 analog output
const int PIN_KY_DO = 33;  // Digital input: KY-036 digital output
const int PIN_R = 27;      // RGB red channel   (series resistor 220-330 ohm)
const int PIN_G = 26;      // RGB green channel
const int PIN_B = 25;      // RGB blue channel

// ---------- PWM (LEDC peripheral) ----------
const int CH_R = 0, CH_G = 1, CH_B = 2;  // Independent LEDC channels
const int PWM_FREQ = 1000;               // 1 kHz: no visible flicker
const int PWM_RES  = 8;                  // 8-bit duty (0-255), matches RGB bytes

// ---------- Telemetry timing ----------
const unsigned long TX_PERIOD_MS = 100;  // 10 Hz telemetry rate
unsigned long lastTx = 0;                // Timestamp of the last transmission

String rxBuffer;                         // Accumulates incoming serial bytes

// Write an 8-bit duty on each color channel.
// For a COMMON ANODE LED invert the duty: ledcWrite(CH_R, 255 - r); etc.
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(CH_R, r);
  ledcWrite(CH_G, g);
  ledcWrite(CH_B, b);
}

// Parse one received JSON line and execute the command it carries.
void handleCommand(const String &line) {
  JsonDocument doc;                                  // Elastic document (ArduinoJson v7)
  DeserializationError err = deserializeJson(doc, line);
  if (err) return;                                   // Ignore malformed lines silently

  const char *cmd = doc["cmd"];                      // Extract the command name
  if (cmd == nullptr) return;

  if (strcmp(cmd, "rgb") == 0) {
    // Missing keys default to 0 thanks to the | operator
    setRGB(doc["r"] | 0, doc["g"] | 0, doc["b"] | 0);
  } else if (strcmp(cmd, "off") == 0) {
    setRGB(0, 0, 0);
  }
}

void setup() {
  Serial.begin(115200);                  // USB-serial link with the Python server

  pinMode(PIN_KY_DO, INPUT);             // KY-036 digital line

  // Full-scale attenuation (~0-3.3 V) on every ADC pin used
  analogSetPinAttenuation(PIN_LM35,  ADC_11db);
  analogSetPinAttenuation(PIN_HALL,  ADC_11db);
  analogSetPinAttenuation(PIN_KY_AO, ADC_11db);

  // Configure the three PWM channels and bind them to the LED pins
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(CH_G, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_R, CH_R);
  ledcAttachPin(PIN_G, CH_G);
  ledcAttachPin(PIN_B, CH_B);

  setRGB(0, 0, 0);                       // Start with the LED off
}

void loop() {
  // ---- 1. Receive commands (non-blocking, line oriented) ----
  while (Serial.available() > 0) {
    char c = (char)Serial.read();        // Consume one byte from the RX buffer
    if (c == '\n') {                     // End of line -> complete command
      handleCommand(rxBuffer);
      rxBuffer = "";                     // Reset accumulator for the next line
    } else if (c != '\r') {
      rxBuffer += c;                     // Append byte, ignoring carriage returns
    }
  }

  // ---- 2. Periodic telemetry ----
  unsigned long now = millis();
  if (now - lastTx >= TX_PERIOD_MS) {
    lastTx = now;

    // analogReadMilliVolts applies the factory ADC calibration and returns mV
    float tempC = analogReadMilliVolts(PIN_LM35) / 10.0f;    // LM35: 10 mV per Celsius
    float vHall = analogReadMilliVolts(PIN_HALL) / 1000.0f;  // Line voltage in volts
    float vKy   = analogReadMilliVolts(PIN_KY_AO) / 1000.0f;
    int   kyDig = digitalRead(PIN_KY_DO);                    // 1 when touch detected

    // Build and emit one JSON telemetry line
    JsonDocument doc;
    doc["t"]   = ((int)(tempC * 10)) / 10.0f;  // Round to one decimal
    doc["vh"]  = ((int)(vHall * 100)) / 100.0f;
    doc["ky"]  = kyDig;
    doc["vky"] = ((int)(vKy * 100)) / 100.0f;
    serializeJson(doc, Serial);
    Serial.println();                          // Line terminator expected by the server
  }
}
