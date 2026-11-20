// SPDX-License-Identifier: Apache-2.0
/*
  esp32_led.ino  --  FEAGI-driven LED on an ESP32 (Arduino core 2.x or 3.x)

  Host (controller.py) sends one line per change over USB serial:
      "<brightness>\n"     brightness 0..100   (0 = off, 100 = full)
  Extra commands (type them in the Serial Monitor to test without FEAGI):
      "t\n"  self-test: blink 3x
      "?\n"  print current brightness

  Wiring (default):  GPIO2 -> 220-330 ohm resistor -> LED anode(+, long leg)
                     LED cathode(-, short leg) -> GND
  GPIO2 is also the blue on-board LED on most ESP32 DevKit V1 boards, so you
  can test with no wiring at all.  Change LED_PIN to use another pin
  (safe choices: 4, 13, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33).
*/

#include <Arduino.h>

const int  LED_PIN     = 2;
const bool ACTIVE_LOW  = false;   // true if your LED is wired 3V3 -> LED -> pin
const int  PWM_FREQ    = 5000;
const int  PWM_BITS    = 8;       // duty 0..255
const int  PWM_CHANNEL = 0;       // only used on Arduino-ESP32 core 2.x
const unsigned long BAUD = 115200;

int brightness = 0;               // 0..100
String line;

void writeDuty(int pct) {
  pct = constrain(pct, 0, 100);
  int duty = map(pct, 0, 100, 0, (1 << PWM_BITS) - 1);
  if (ACTIVE_LOW) duty = ((1 << PWM_BITS) - 1) - duty;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LED_PIN, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

void setBrightness(int pct) {
  brightness = constrain(pct, 0, 100);
  writeDuty(brightness);
}

void selfTest() {
  for (int i = 0; i < 3; i++) {
    writeDuty(100); delay(200);
    writeDuty(0);   delay(200);
  }
  writeDuty(brightness);
  Serial.println("OK selftest");
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  if (s == "t") { selfTest(); return; }
  if (s == "?") { Serial.print("B "); Serial.println(brightness); return; }
  // Accept "57" or "57,..." (anything after a comma is ignored)
  int comma = s.indexOf(',');
  if (comma >= 0) s = s.substring(0, comma);
  setBrightness(s.toInt());
}

void setup() {
  Serial.begin(BAUD);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LED_PIN, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_BITS);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
#endif
  setBrightness(0);
  delay(200);
  selfTest();                     // 3 blinks on boot = firmware alive
  Serial.println("READY esp32_led");
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleLine(line);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
}
