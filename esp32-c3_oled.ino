/*
  ESP32-C3 Charger - Improved with PI controller and anti-windup.
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
const int PWM_OUT_PIN = 10;               // LEDC PWM output to MOSFET gate
const int BAT_VOLTAGE_SENSE_PIN = 0;      // ADC input for battery voltage (GPIO number)
const int CURRENT_SENSE_AMP_PIN = 1;      // ADC input for ACS712 output (GPIO number)
const int DESIRED_VOLTAGE_SET_PIN = 2;    // ADC input for setpoint pot (voltage)
const int DESIRED_CURRENT_SET_PIN = 3;    // ADC input for setpoint pot (current)
const int CHARGED_INDICATOR_PIN = 9;      // Digital output indicator (LED)

// --- PWM (LEDC) settings ---
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;    // 50 kHz
const int PWM_RESOLUTION = 10; // bits (0..1023)
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;

// --- ADC / Sampling ---
const float ADC_MAX_VOLTAGE = 3.3f;  // Vref
const float ADC_MAX_READING = 4095.0f; // 12-bit
const int ADC_SAMPLES = 16;
const int CALIBRATION_SAMPLES = 100;

// --- Calibration ---
const float VOLTAGE_DIVIDER_RATIO = 9.33f;
const float ACS712_SENSITIVITY = 0.185f;

const float ADC_V_PER_COUNT = (ADC_MAX_VOLTAGE / ADC_MAX_READING);
const float VOLTAGE_SENSE_FACTOR = ADC_V_PER_COUNT * VOLTAGE_DIVIDER_RATIO;
const float CURRENT_RAW_TO_A = ADC_V_PER_COUNT / ACS712_SENSITIVITY;

// --- Charger State Machine ---
enum ChargerState_t { IDLE, CHARGING, PAUSED_CHECK_VOLTAGE, CHARGED_COMPLETE };
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

// --- PI Controller parameters ---
float KP_CC = 15.0f;
float KI_CC = 4.0f;
float KP_CV = 8.0f;
float KI_CV = 1.5f;

float integralCC = 0.0f;
float integralCV = 0.0f;

// Safety
const float MAX_ALLOWED_CURRENT_MULTIPLIER = 2.0f;

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
int piController(float error, float &integral, float kp, float ki);

// ---------------------- PI Controller ----------------------
int piController(float error, float &integral, float kp, float ki) {
    integral += ki * error;

    // Anti-windup: clamp integral to PWM range
    if (integral > MAX_PWM_DUTY) integral = (float)MAX_PWM_DUTY;
    else if (integral < 0) integral = 0;

    float u = kp * error + integral;
    return constrain((int)u, 0, MAX_PWM_DUTY);
}

// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n=== ESP32 Charger Improved ===");

  pinMode(CHARGED_INDICATOR_PIN, OUTPUT);
  digitalWrite(CHARGED_INDICATOR_PIN, LOW);

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
  delay(50);
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

  batteryVoltage = rawBat * VOLTAGE_SENSE_FACTOR;
  chargingCurrent = (rawI - currentSensorOffsetRaw) * CURRENT_RAW_TO_A;

  desiredFinalVoltage = rawSetV * ADC_V_PER_COUNT * (30.0f / 3.3f);
  desiredCurrentLimit = rawSetI * ADC_V_PER_COUNT * (5.0f / 3.3f);

  desiredFinalVoltage = constrain(desiredFinalVoltage, 26.0f, 30.0f);
  desiredCurrentLimit = constrain(desiredCurrentLimit, 0.1f, 5.0f);

  batteryVoltageFiltered = (batteryVoltageFiltered * (1.0f - FILTER_ALPHA_V)) + (batteryVoltage * FILTER_ALPHA_V);
  chargingCurrentFiltered = (chargingCurrentFiltered * (1.0f - FILTER_ALPHA_I)) + (chargingCurrent * FILTER_ALPHA_I);
}

void setPwmDutyCycle(int dutyCycle) {
  currentPwmDutyCycle = constrain(dutyCycle, 0, MAX_PWM_DUTY);
  ledcWrite(PWM_CHANNEL, currentPwmDutyCycle);
}

bool wasInCV = false;

void updateChargerState() {
  unsigned long now = millis();

  switch (chargerState) {
    case IDLE:
      setPwmDutyCycle(0);
      integralCC = 0; integralCV = 0; wasInCV = false;
      if (batteryVoltageFiltered < (desiredFinalVoltage - 1.0f)) {
        chargerState = CHARGING;
        lastChargeCheckTime = now;
        digitalWrite(CHARGED_INDICATOR_PIN, LOW);
      }
      break;

    case CHARGING: {
      // Safety: measured current exceeds limit by too much
      if (chargingCurrentFiltered > (desiredCurrentLimit * MAX_ALLOWED_CURRENT_MULTIPLIER + 0.5f)) {
        Serial.println("EMERGENCY SHUTDOWN: Overcurrent");
        setPwmDutyCycle(0);
        chargerState = IDLE;
        break;
      }

      int dutyCmd;
      // Use a small hysteresis for CC/CV transition to avoid chattering
      bool useCV = batteryVoltageFiltered >= (desiredFinalVoltage - 0.05f);
      if (wasInCV) {
          useCV = batteryVoltageFiltered >= (desiredFinalVoltage - 0.3f);
      }

      if (useCV) {
          // CV Mode
          if (!wasInCV) {
              // Bumpless transfer: initialize CV integral with current duty cycle
              integralCV = (float)currentPwmDutyCycle;
              wasInCV = true;
          }
          float errV = desiredFinalVoltage - batteryVoltageFiltered;
          dutyCmd = piController(errV, integralCV, KP_CV, KI_CV);
      } else {
          // CC Mode / Bulk
          if (wasInCV) {
              // Bumpless transfer: initialize CC integral with current duty cycle
              integralCC = (float)currentPwmDutyCycle;
              wasInCV = false;
          }
          float errI = desiredCurrentLimit - chargingCurrentFiltered;
          dutyCmd = piController(errI, integralCC, KP_CC, KI_CC);
      }

      setPwmDutyCycle(dutyCmd);

      if (now - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
        setPwmDutyCycle(0);
        pausedStartTime = now;
        chargerState = PAUSED_CHECK_VOLTAGE;
      }
      break;
    }

    case PAUSED_CHECK_VOLTAGE:
      if (now - pausedStartTime < PAUSE_SETTLE_MS) return;

      // Update readings after settling
      readSensorInputs();
      // We check raw batteryVoltage here for accuracy since it's unloaded
      if (batteryVoltage >= desiredFinalVoltage - 0.1f) {
        chargerState = CHARGED_COMPLETE;
        digitalWrite(CHARGED_INDICATOR_PIN, HIGH);
      } else {
        chargerState = CHARGING;
        lastChargeCheckTime = now;
        // Don't reset integrals here to maintain duty cycle when resuming
      }
      break;

    case CHARGED_COMPLETE:
      setPwmDutyCycle(0);
      if (batteryVoltageFiltered <= (desiredFinalVoltage - 0.5f)) {
        chargerState = CHARGING;
        digitalWrite(CHARGED_INDICATOR_PIN, LOW);
        lastChargeCheckTime = now;
      }
      break;
  }
}

void updateOLED() {
  if (!oledAvailable) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("State: ");
  switch (chargerState) {
    case IDLE: display.println("IDLE"); break;
    case CHARGING: display.println("CHARGING"); break;
    case PAUSED_CHECK_VOLTAGE: display.println("PAUSED"); break;
    case CHARGED_COMPLETE: display.println("CHARGED!"); break;
  }
  display.print("Bat V: "); display.print(batteryVoltageFiltered, 2); display.println(" V");
  display.print("Chg I: "); display.print(chargingCurrentFiltered, 3); display.println(" A");
  display.print("PWM: "); display.println(currentPwmDutyCycle);
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
