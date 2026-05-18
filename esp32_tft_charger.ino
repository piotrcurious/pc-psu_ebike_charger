/*
  ESP32-WROOM Charger - TFT Version.
  Uses LovyanGFX for 320x240 SPI Display.
*/

#include "config_esp32.h"
#include "Charger.h"
#include "UI_TFT.h"

LGFX tft;
Charger charger;
UI_TFT ui(tft, charger);

void setup() {
  Serial.begin(115200);
  delay(10);

  charger.setup();
  ui.setup();
}

void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long now = millis();

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

  // Adaptive delay to maintain loop timing
  unsigned long loopEnd = millis();
  unsigned long elapsed = loopEnd - now;
  if (elapsed < 20) delay(20 - elapsed);
}
