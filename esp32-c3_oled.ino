/*
  ESP32-C3 Charger - Modular Version.
*/

#include "config.h"
#include "Charger.h"
#include "UI.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Charger charger;
UI ui(display, charger);

void setup() {
  Serial.begin(115200);
  delay(10);

  // Initialize I2C for OLED on ESP32-C3 specified pins
  Wire.begin(SDA_PIN, SCL_PIN);

  charger.setup();
  ui.setup();
}

void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long now = millis();

  // Calculate dt, handle first loop
  float dt;
  if (lastLoopTime == 0) {
      dt = 0.02f;
  } else {
      dt = (float)(now - lastLoopTime) / 1000.0f;
  }
  lastLoopTime = now;

  charger.update(dt);
  ui.update();

  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'c' || c == 'C') charger.calibrateCurrentSensor();
    if (c == 'r' || c == 'R') charger.reset();
  }

  delay(20);
}
