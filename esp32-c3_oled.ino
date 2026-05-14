/*
  ESP32-C3 Charger - Robust Step-Based Version with ATX PSU Control.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Display Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledAvailable = false;

// --- Pin Definitions (GPIO numbers) ---
const int PWM_OUT_PIN = 10;
const int BAT_VOLTAGE_SENSE_PIN = 0;
const int CURRENT_SENSE_AMP_PIN = 1;
const int DESIRED_VOLTAGE_SET_PIN = 2;
const int DESIRED_CURRENT_SET_PIN = 3;
const int ATX_PS_ON_PIN = 9;

// --- PWM (LEDC) settings ---
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 10;
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;

// --- ADC / Sampling ---
const float ADC_MAX_VOLTAGE = 3.3f;
const float ADC_MAX_READING = 4095.0f;
const int ADC_SAMPLES = 16;
const int CALIBRATION_SAMPLES = 100;

// --- Calibration ---
const float VOLTAGE_DIVIDER_RATIO = 9.33f;
const float ACS712_SENSITIVITY = 0.185f;

const float ADC_V_PER_COUNT = (ADC_MAX_VOLTAGE / ADC_MAX_READING);
const float VOLTAGE_SENSE_FACTOR = ADC_V_PER_COUNT * VOLTAGE_DIVIDER_RATIO;
const float CURRENT_RAW_TO_A = ADC_V_PER_COUNT / ACS712_SENSITIVITY;

// --- Charger State Machine ---
enum ChargerState_t { IDLE, CHARGING, PAUSED_CHECK_VOLTAGE, CHARGED_COMPLETE, ERROR_STATE };
ChargerState_t chargerState = IDLE;

// --- Global Variables ---
float batteryVoltage = 0.0f;
float batteryVoltageFiltered = 0.0f;
float chargingCurrent = 0.0f;
float chargingCurrentFiltered = 0.0f;
float desiredFinalVoltage = 29.0f;
float desiredCurrentLimit = 1.0f;
int currentPwmDutyCycle = 0;

int currentSensorOffsetRaw = 2048;

// --- Timing ---
unsigned long lastChargeCheckTime = 0;
const unsigned long CHARGE_CHECK_INTERVAL_MS = 30000UL;
unsigned long pausedStartTime = 0;
const unsigned long PAUSE_SETTLE_MS = 1000UL;

unsigned long lastPsuOffTime = 0;
const unsigned long PSU_RESTART_COOLDOWN_MS = 10000UL;

unsigned long chargeStartTime = 0;
const unsigned long MAX_CHARGE_TIME_MS = 12 * 3600 * 1000UL;

// --- Control Strategy Parameters ---
const int PWM_STEP_UP = 1;
const int PWM_STEP_DOWN_FAST = 5;
const int PWM_STEP_DOWN_SLOW = 1;

const float VOLTAGE_DEADBAND = 0.05f;
const float CURRENT_DEADBAND = 0.05f;

const float MIN_BATTERY_VOLTAGE = 15.0f;
const float MAX_ALLOWED_CURRENT_MULTIPLIER = 2.0f;

// Soft-start
float targetCurrentLimit = 0.0f;
const float SOFT_START_RAMP_A_PER_S = 0.1f;

// Charge Termination in CV mode
const float FULL_CHARGE_CURRENT_THRESHOLD = 0.1f; // A

// Filters
const float FILTER_ALPHA_V = 0.2f;
const float FILTER_ALPHA_I = 0.2f;

// Function prototypes
void readSensorInputs();
void updateChargerState();
void updateOLED();
void setPwmDutyCycle(int dutyCycle);
int analogReadAveraged(int pin);
void calibrateCurrentSensor(bool verbose = true);
void enableOledOrWarn();
void setPsuState(bool on);

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
  batteryVoltageFiltered = batteryVoltage;
  chargingCurrentFiltered = chargingCurrent;

  lastChargeCheckTime = millis();
}

void loop() {
  readSensorInputs();
  updateChargerState();
  updateOLED();

  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'c' || c == 'C') calibrateCurrentSensor(true);
  }
  delay(20);
}

int analogReadAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    sum += analogRead(pin);
    delayMicroseconds(200);
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

  desiredFinalVoltage = (float)rawSetV * ADC_V_PER_COUNT * (30.0f / 3.3f);
  float rawCurrentLimit = (float)rawSetI * ADC_V_PER_COUNT * (5.0f / 3.3f);

  desiredFinalVoltage = constrain(desiredFinalVoltage, 26.0f, 30.0f);
  desiredCurrentLimit = constrain(rawCurrentLimit, 0.1f, 5.0f);

  batteryVoltageFiltered = (batteryVoltageFiltered * (1.0f - FILTER_ALPHA_V)) + (batteryVoltage * FILTER_ALPHA_V);
  chargingCurrentFiltered = (chargingCurrentFiltered * (1.0f - FILTER_ALPHA_I)) + (chargingCurrent * FILTER_ALPHA_I);
}

void setPwmDutyCycle(int dutyCycle) {
  currentPwmDutyCycle = constrain(dutyCycle, 0, MAX_PWM_DUTY);
  ledcWrite(PWM_CHANNEL, currentPwmDutyCycle);
}

void setPsuState(bool on) {
    bool currentState = (digitalRead(ATX_PS_ON_PIN) == HIGH);
    if (on) {
        if (!currentState) {
            unsigned long now = millis();
            if (now - lastPsuOffTime >= PSU_RESTART_COOLDOWN_MS) {
                digitalWrite(ATX_PS_ON_PIN, HIGH);
                Serial.println("PSU turned ON");
            }
        }
    } else {
        if (currentState) {
            digitalWrite(ATX_PS_ON_PIN, LOW);
            lastPsuOffTime = millis();
            Serial.println("PSU turned OFF");
        }
    }
}

void updateChargerState() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (float)(now - lastUpdate) / 1000.0f;
  if (chargerState != PAUSED_CHECK_VOLTAGE && now - lastUpdate < 50) return;
  lastUpdate = now;

  switch (chargerState) {
    case IDLE:
      setPwmDutyCycle(0);
      setPsuState(false);
      targetCurrentLimit = 0.05f;
      if (batteryVoltageFiltered > MIN_BATTERY_VOLTAGE && batteryVoltageFiltered < (desiredFinalVoltage - 1.0f)) {
        setPsuState(true);
        if (digitalRead(ATX_PS_ON_PIN) == HIGH) {
            chargerState = CHARGING;
            lastChargeCheckTime = now;
            chargeStartTime = now;
            Serial.println("State: IDLE -> CHARGING");
        }
      }
      break;

    case CHARGING: {
      if (now - chargeStartTime > MAX_CHARGE_TIME_MS) {
        Serial.println("ERROR: Charge timeout");
        chargerState = ERROR_STATE;
        break;
      }

      setPsuState(true);
      if (digitalRead(ATX_PS_ON_PIN) == LOW) break;

      if (targetCurrentLimit < desiredCurrentLimit) {
          targetCurrentLimit += SOFT_START_RAMP_A_PER_S * dt;
          if (targetCurrentLimit > desiredCurrentLimit) targetCurrentLimit = desiredCurrentLimit;
      } else {
          targetCurrentLimit = desiredCurrentLimit;
      }

      if (chargingCurrentFiltered > (desiredCurrentLimit * MAX_ALLOWED_CURRENT_MULTIPLIER + 0.5f)) {
        Serial.println("EMERGENCY SHUTDOWN: Overcurrent");
        chargerState = ERROR_STATE;
        break;
      }

      int nextPwm = currentPwmDutyCycle;
      if (chargingCurrentFiltered > (targetCurrentLimit + CURRENT_DEADBAND)) {
          nextPwm -= PWM_STEP_DOWN_FAST;
      }
      else if (batteryVoltageFiltered > (desiredFinalVoltage + VOLTAGE_DEADBAND)) {
          nextPwm -= PWM_STEP_DOWN_SLOW;
      }
      else if (chargingCurrentFiltered < (targetCurrentLimit - CURRENT_DEADBAND) &&
               batteryVoltageFiltered < (desiredFinalVoltage - VOLTAGE_DEADBAND)) {
          nextPwm += PWM_STEP_UP;
      }
      setPwmDutyCycle(nextPwm);

      // Check termination: in CV mode and current has dropped low
      if (batteryVoltageFiltered >= (desiredFinalVoltage - 0.1f) && chargingCurrentFiltered < FULL_CHARGE_CURRENT_THRESHOLD) {
          static unsigned long fullConditionStart = 0;
          if (fullConditionStart == 0) fullConditionStart = now;
          if (now - fullConditionStart > 10000) { // Stable for 10s
             Serial.println("Battery full - State: CHARGED_COMPLETE");
             setPsuState(false);
             chargerState = CHARGED_COMPLETE;
             fullConditionStart = 0;
             break;
          }
      }

      if (now - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
        Serial.println("Periodic check - pausing PWM");
        setPwmDutyCycle(0);
        pausedStartTime = now;
        chargerState = PAUSED_CHECK_VOLTAGE;
      }
      break;
    }

    case PAUSED_CHECK_VOLTAGE:
      setPwmDutyCycle(0);
      if (now - pausedStartTime < PAUSE_SETTLE_MS) return;

      readSensorInputs();
      if (batteryVoltage >= desiredFinalVoltage - 0.1f) {
        Serial.println("Final voltage reached - State: CHARGED_COMPLETE");
        setPsuState(false);
        chargerState = CHARGED_COMPLETE;
      } else if (batteryVoltage < MIN_BATTERY_VOLTAGE) {
        Serial.println("ERROR: Battery disconnected");
        chargerState = IDLE;
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
      if (batteryVoltageFiltered > MIN_BATTERY_VOLTAGE && batteryVoltageFiltered <= (desiredFinalVoltage - 0.5f)) {
        setPsuState(true);
        if (digitalRead(ATX_PS_ON_PIN) == HIGH) {
            chargerState = CHARGING;
            lastChargeCheckTime = now;
            chargeStartTime = now;
            Serial.println("Voltage dropped - State: CHARGING");
        }
      } else if (batteryVoltageFiltered < MIN_BATTERY_VOLTAGE) {
          chargerState = IDLE;
      }
      break;

    case ERROR_STATE:
      setPwmDutyCycle(0);
      setPsuState(false);
      break;
  }
}

void updateOLED() {
  if (!oledAvailable) return;
  display.clearDisplay();

  // Header: State
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  switch (chargerState) {
    case IDLE: display.print("IDLE"); break;
    case CHARGING: display.print("CHARGING"); break;
    case PAUSED_CHECK_VOLTAGE: display.print("PAUSED"); break;
    case CHARGED_COMPLETE: display.print("CHARGED!"); break;
    case ERROR_STATE: display.print("ERROR!"); break;
  }

  // Elapsed Time
  if (chargerState == CHARGING || chargerState == PAUSED_CHECK_VOLTAGE || chargerState == CHARGED_COMPLETE) {
      unsigned long elapsed = (chargerState == CHARGED_COMPLETE) ? (lastChargeCheckTime - chargeStartTime) : (millis() - chargeStartTime);
      int hours = elapsed / 3600000;
      int mins = (elapsed % 3600000) / 60000;
      display.setCursor(70, 0);
      if (hours < 10) display.print("0");
      display.print(hours);
      display.print(":");
      if (mins < 10) display.print("0");
      display.print(mins);
  }

  // Large Battery Voltage
  display.setTextSize(2);
  display.setCursor(0, 12);
  display.print(batteryVoltageFiltered, 1);
  display.print("V");

  // Current reading
  display.setTextSize(1);
  display.setCursor(70, 12);
  display.print(chargingCurrentFiltered, 2);
  display.print("A");
  display.setCursor(70, 20);
  display.print("Lim:");
  display.print(desiredCurrentLimit, 1);

  // PSU State and Cooldown
  display.setCursor(0, 32);
  if (digitalRead(ATX_PS_ON_PIN) == HIGH) {
      display.print("PSU: ON");
  } else {
      unsigned long now = millis();
      if (now - lastPsuOffTime < PSU_RESTART_COOLDOWN_MS) {
          int remaining = (PSU_RESTART_COOLDOWN_MS - (now - lastPsuOffTime)) / 1000;
          display.print("PSU: COOLDOWN ");
          display.print(remaining);
      } else {
          display.print("PSU: OFF");
      }
  }

  // Progress Bar
  float minV = desiredFinalVoltage - 5.0f;
  if (minV < 15.0f) minV = 15.0f;
  int progress = (int)((batteryVoltageFiltered - minV) / (desiredFinalVoltage - minV) * 100.0f);
  progress = constrain(progress, 0, 100);

  display.drawRect(0, 44, 102, 10, SSD1306_WHITE);
  display.fillRect(2, 46, (int)(progress * 0.98f), 6, SSD1306_WHITE);
  display.setCursor(105, 46);
  display.print(progress);
  display.print("%");

  // PWM info at bottom
  display.setCursor(0, 56);
  display.print("PWM: ");
  display.print(currentPwmDutyCycle);
  display.print("/");
  display.print(MAX_PWM_DUTY);

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
