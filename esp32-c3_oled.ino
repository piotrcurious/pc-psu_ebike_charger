/*
  ESP32-C3 Charger - Robust Step-Based Version with ATX PSU Control.
*/

#include "config.h"
#include "Filter.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Global Objects ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledAvailable = false;

EMAFilter<float> vBatFilter(FILTER_ALPHA_V);
EMAFilter<float> iChgFilter(FILTER_ALPHA_I);

// --- Charger State Machine ---
enum ChargerState_t { IDLE, CHARGING, PAUSED_CHECK_VOLTAGE, CHARGED_COMPLETE, ERROR_STATE };
ChargerState_t chargerState = IDLE;

enum ErrorType_t { NO_ERROR, OVERCURRENT, TIMEOUT, DISCONNECTED };
ErrorType_t lastError = NO_ERROR;

// --- Global Variables ---
float batteryVoltage = 0.0f;
float chargingCurrent = 0.0f;
float desiredFinalVoltage = 29.0f;
float desiredCurrentLimit = 1.0f;
int currentPwmDutyCycle = 0;

int currentSensorOffsetRaw = 2048;

unsigned long lastChargeCheckTime = 0;
unsigned long pausedStartTime = 0;
unsigned long lastPsuOffTime = 0;
unsigned long chargeStartTime = 0;
unsigned long chargeEndTime = 0;
unsigned long lastDiagTime = 0;

float targetCurrentLimit = 0.05f;

// --- Function Prototypes ---
void readSensorInputs();
void updateChargerState();
void updateOLED();
void setPwmDutyCycle(int dutyCycle);
int analogReadAveraged(int pin);
void calibrateCurrentSensor(bool verbose = true);
void enableOledOrWarn();
void setPsuState(bool on);
void handleChargingState(unsigned long now, float dt);
void handleError(ErrorType_t type, const char* msg);
void outputDiagnostics(unsigned long now);

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n=== ESP32 Charger Robust Version ===");

  pinMode(ATX_PS_ON_PIN, OUTPUT);
  setPsuState(false);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);
  setPwmDutyCycle(0);

  #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
      analogReadResolution(12);
      analogSetPinAttenuation(BAT_VOLTAGE_SENSE_PIN, ADC_ATTEN_DB_11);
      analogSetPinAttenuation(CURRENT_SENSE_AMP_PIN, ADC_ATTEN_DB_11);
      analogSetPinAttenuation(DESIRED_VOLTAGE_SET_PIN, ADC_ATTEN_DB_11);
      analogSetPinAttenuation(DESIRED_CURRENT_SET_PIN, ADC_ATTEN_DB_11);
  #endif

  Wire.begin(8, 7);
  enableOledOrWarn();

  calibrateCurrentSensor(true);

  readSensorInputs();
  lastChargeCheckTime = millis();
}

// ---------------------- Loop ----------------------
void loop() {
  unsigned long now = millis();
  readSensorInputs();
  updateChargerState();
  updateOLED();
  outputDiagnostics(now);

  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'c' || c == 'C') calibrateCurrentSensor(true);
    if (c == 'r' || c == 'R') {
        chargerState = IDLE;
        lastError = NO_ERROR;
        vBatFilter.reset();
        iChgFilter.reset();
        Serial.println("System Reset");
    }
  }
  delay(20);
}

// ---------------------- Hardware Access ----------------------
int analogReadAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return (int)(sum / ADC_SAMPLES);
}

void calibrateCurrentSensor(bool verbose) {
  setPwmDutyCycle(0);
  setPsuState(false);
  delay(1000);
  long sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    sum += analogRead(CURRENT_SENSE_AMP_PIN);
    delay(2);
  }
  currentSensorOffsetRaw = (int)(sum / CALIBRATION_SAMPLES);
  if (verbose) {
    Serial.print("ACS712 Calibrated. Offset: "); Serial.println(currentSensorOffsetRaw);
  }
}

void readSensorInputs() {
  int rawBat = analogReadAveraged(BAT_VOLTAGE_SENSE_PIN);
  int rawI   = analogReadAveraged(CURRENT_SENSE_AMP_PIN);
  int rawSetV = analogReadAveraged(DESIRED_VOLTAGE_SET_PIN);
  int rawSetI = analogReadAveraged(DESIRED_CURRENT_SET_PIN);

  batteryVoltage = (float)rawBat * VOLTAGE_SENSE_FACTOR;
  chargingCurrent = (float)(rawI - currentSensorOffsetRaw) * CURRENT_RAW_TO_A;

  vBatFilter.update(batteryVoltage);
  iChgFilter.update(chargingCurrent);

  desiredFinalVoltage = (float)rawSetV * ADC_V_PER_COUNT * (30.0f / 3.3f);
  float rawCurrentLimit = (float)rawSetI * ADC_V_PER_COUNT * (5.0f / 3.3f);

  desiredFinalVoltage = constrain(desiredFinalVoltage, 26.0f, 30.0f);
  desiredCurrentLimit = constrain(rawCurrentLimit, 0.1f, 5.0f);
}

void setPwmDutyCycle(int dutyCycle) {
  currentPwmDutyCycle = constrain(dutyCycle, 0, MAX_PWM_DUTY);
  ledcWrite(PWM_CHANNEL, currentPwmDutyCycle);
}

void setPsuState(bool on) {
    bool isActuallyOn = (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON);
    if (on) {
        if (!isActuallyOn) {
            unsigned long now = millis();
            if (now - lastPsuOffTime >= PSU_RESTART_COOLDOWN_MS) {
                digitalWrite(ATX_PS_ON_PIN, ATX_PSU_ON);
                Serial.println("PSU turned ON");
            }
        }
    } else {
        if (isActuallyOn) {
            digitalWrite(ATX_PS_ON_PIN, ATX_PSU_OFF);
            lastPsuOffTime = millis();
            Serial.println("PSU turned OFF");
        }
    }
}

// ---------------------- Diagnostics ----------------------
void outputDiagnostics(unsigned long now) {
    if (now - lastDiagTime >= 5000) { // Every 5 seconds
        lastDiagTime = now;
        Serial.print("DIAG: State="); Serial.print((int)chargerState);
        Serial.print(" Vbat="); Serial.print(vBatFilter.value(), 2);
        Serial.print(" Ichg="); Serial.print(iChgFilter.value(), 3);
        Serial.print(" PWM="); Serial.print(currentPwmDutyCycle);
        Serial.print(" PSU="); Serial.println(digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON ? "ON" : "OFF");
    }
}

// ---------------------- Charger Logic ----------------------
void handleError(ErrorType_t type, const char* msg) {
    Serial.print("ERROR: "); Serial.println(msg);
    lastError = type;
    chargerState = ERROR_STATE;
    setPwmDutyCycle(0);
    setPsuState(false);
}

void updateChargerState() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (float)(now - lastUpdate) / 1000.0f;
  if (chargerState != PAUSED_CHECK_VOLTAGE && now - lastUpdate < 50) return;
  lastUpdate = now;

  float bvFiltered = vBatFilter.value();

  switch (chargerState) {
    case IDLE:
      setPwmDutyCycle(0);
      setPsuState(false);
      targetCurrentLimit = 0.05f;
      if (bvFiltered > MIN_BATTERY_VOLTAGE && bvFiltered < (desiredFinalVoltage - 1.0f)) {
        setPsuState(true);
        if (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON) {
            chargerState = CHARGING;
            lastChargeCheckTime = now;
            chargeStartTime = now;
            Serial.println("State: IDLE -> CHARGING");
        }
      }
      break;

    case CHARGING:
      handleChargingState(now, dt);
      break;

    case PAUSED_CHECK_VOLTAGE:
      setPwmDutyCycle(0);
      if (now - pausedStartTime < PAUSE_SETTLE_MS) return;
      readSensorInputs();
      if (batteryVoltage >= desiredFinalVoltage - 0.1f) {
        Serial.println("Final voltage reached - State: CHARGED_COMPLETE");
        chargeEndTime = now;
        setPsuState(false);
        chargerState = CHARGED_COMPLETE;
      } else if (batteryVoltage < MIN_BATTERY_VOLTAGE) {
        handleError(DISCONNECTED, "Battery disconnected");
      } else {
        Serial.println("Voltage not reached - State: CHARGING");
        chargerState = CHARGING;
        lastChargeCheckTime = now;
      }
      break;

    case CHARGED_COMPLETE:
      setPwmDutyCycle(0);
      setPsuState(false);
      targetCurrentLimit = 0.05f;
      if (bvFiltered > MIN_BATTERY_VOLTAGE && bvFiltered <= (desiredFinalVoltage - 0.5f)) {
        setPsuState(true);
        if (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON) {
            chargerState = CHARGING;
            lastChargeCheckTime = now;
            chargeStartTime = now;
            Serial.println("Voltage dropped - State: CHARGING");
        }
      } else if (bvFiltered < MIN_BATTERY_VOLTAGE) {
          chargerState = IDLE;
      }
      break;

    case ERROR_STATE:
      setPwmDutyCycle(0);
      setPsuState(false);
      break;
  }
}

void handleChargingState(unsigned long now, float dt) {
    if (now - chargeStartTime > MAX_CHARGE_TIME_MS) {
        handleError(TIMEOUT, "Charge timeout");
        return;
    }

    setPsuState(true);
    if (digitalRead(ATX_PS_ON_PIN) != ATX_PSU_ON) return;

    if (targetCurrentLimit < desiredCurrentLimit) {
        targetCurrentLimit += SOFT_START_RAMP_A_PER_S * dt;
        if (targetCurrentLimit > desiredCurrentLimit) targetCurrentLimit = desiredCurrentLimit;
    } else {
        targetCurrentLimit = desiredCurrentLimit;
    }

    float iFiltered = iChgFilter.value();
    float bvFiltered = vBatFilter.value();

    if (iFiltered > (desiredCurrentLimit * MAX_ALLOWED_CURRENT_MULTIPLIER + 0.5f)) {
        handleError(OVERCURRENT, "Overcurrent");
        return;
    }

    int nextPwm = currentPwmDutyCycle;
    if (iFiltered > (targetCurrentLimit + CURRENT_DEADBAND)) {
        nextPwm -= PWM_STEP_DOWN_FAST;
    }
    else if (bvFiltered > (desiredFinalVoltage + VOLTAGE_DEADBAND)) {
        nextPwm -= PWM_STEP_DOWN_SLOW;
    }
    else if (iFiltered < (targetCurrentLimit - CURRENT_DEADBAND) &&
             bvFiltered < (desiredFinalVoltage - VOLTAGE_DEADBAND)) {
        nextPwm += PWM_STEP_UP;
    }
    setPwmDutyCycle(nextPwm);

    static unsigned long fullConditionStart = 0;
    if (bvFiltered >= (desiredFinalVoltage - 0.1f) && iFiltered < FULL_CHARGE_CURRENT_THRESHOLD) {
        if (fullConditionStart == 0) fullConditionStart = now;
        if (now - fullConditionStart > 10000) {
            Serial.println("Battery full - State: CHARGED_COMPLETE");
            chargeEndTime = now;
            setPsuState(false);
            chargerState = CHARGED_COMPLETE;
            fullConditionStart = 0;
            return;
        }
    } else {
        fullConditionStart = 0;
    }

    if (now - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
        Serial.println("Periodic check - pausing PWM");
        setPwmDutyCycle(0);
        pausedStartTime = now;
        chargerState = PAUSED_CHECK_VOLTAGE;
    }
}

// ---------------------- UI / OLED ----------------------
void updateOLED() {
  if (!oledAvailable) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Header
  display.setTextSize(1);
  display.setCursor(0, 0);
  switch (chargerState) {
    case IDLE: display.print("IDLE"); break;
    case CHARGING: display.print("CHARGING"); break;
    case PAUSED_CHECK_VOLTAGE: display.print("PAUSED"); break;
    case CHARGED_COMPLETE: display.print("CHARGED!"); break;
    case ERROR_STATE: display.print("ERROR!"); break;
  }

  unsigned long now = millis();
  float bvFiltered = vBatFilter.value();
  float iFiltered = iChgFilter.value();

  // Elapsed Time
  if (chargerState == CHARGING || chargerState == PAUSED_CHECK_VOLTAGE || chargerState == CHARGED_COMPLETE || chargerState == ERROR_STATE) {
      unsigned long elapsed;
      if (chargerState == CHARGING || chargerState == PAUSED_CHECK_VOLTAGE) elapsed = now - chargeStartTime;
      else elapsed = chargeEndTime - chargeStartTime;

      int hours = elapsed / 3600000;
      int mins = (elapsed % 3600000) / 60000;
      display.setCursor(70, 0);
      if (hours < 10) display.print("0");
      display.print(hours);
      display.print(":");
      if (mins < 10) display.print("0");
      display.print(mins);
  }

  if (chargerState == ERROR_STATE) {
      display.setCursor(0, 16);
      display.setTextSize(1);
      switch(lastError) {
          case OVERCURRENT: display.println("OVERCURRENT"); break;
          case TIMEOUT: display.println("TIMEOUT"); break;
          case DISCONNECTED: display.println("DISCONNECTED"); break;
          default: display.println("UNKNOWN ERROR"); break;
      }
      display.println("\nReset required.");
  } else if (chargerState == CHARGED_COMPLETE) {
      display.setCursor(0, 16);
      display.setTextSize(1);
      display.println("Charging Summary:");
      display.print("Final V: "); display.print(bvFiltered, 2); display.println("V");
      unsigned long totalSecs = (chargeEndTime - chargeStartTime) / 1000;
      display.print("Time: "); display.print(totalSecs / 60); display.println(" mins");
  } else {
      // Normal display
      display.setTextSize(2);
      display.setCursor(0, 12);
      display.print(bvFiltered, 1); display.print("V");

      display.setTextSize(1);
      display.setCursor(70, 12);
      display.print(iFiltered, 2); display.print("A");
      display.setCursor(70, 20);
      display.print("Lim:"); display.print(desiredCurrentLimit, 1);

      display.setCursor(0, 32);
      if (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON) {
          display.print("PSU: ON");
      } else {
          if (now - lastPsuOffTime < PSU_RESTART_COOLDOWN_MS) {
              display.print("PSU: COOL "); display.print((PSU_RESTART_COOLDOWN_MS - (now - lastPsuOffTime)) / 1000);
          } else {
              display.print("PSU: OFF");
          }
      }

      // Progress Bar
      float minV = desiredFinalVoltage - 5.0f;
      if (minV < 15.0f) minV = 15.0f;
      int progress = (int)((bvFiltered - minV) / (desiredFinalVoltage - minV) * 100.0f);
      progress = constrain(progress, 0, 100);
      display.drawRect(0, 44, 102, 10, SSD1306_WHITE);
      display.fillRect(2, 46, (int)(progress * 0.98f), 6, SSD1306_WHITE);
      display.setCursor(105, 46);
      display.print(progress); display.print("%");

      display.setCursor(0, 56);
      display.print("PWM: "); display.print(currentPwmDutyCycle);
      display.print("/"); display.print(MAX_PWM_DUTY);
  }

  display.display();
}

void enableOledOrWarn() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledAvailable = false;
  } else {
    oledAvailable = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.println("Ready");
    display.display();
  }
}
