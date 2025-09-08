/*
  ESP32-C3 Charger - improved version with ACS712 auto-calibration at startup

  Key features:
  - ACS712 current sensor zero-offset auto-calibrated at startup (requires NO current during boot).
  - Runtime recalibration via serial (send 'c').
  - ADC multi-sample averaging for noise reduction.
  - Small low-pass smoothing for displayed values.
  - Simple proportional CC/CV control mapped to PWM duty.
  - OLED is optional (skips/hard-fails gracefully if not present).
  - Safety clamps / emergency shutdown if current wildly exceeds limits.

  HARDWARE / CALIBRATION NOTES:
  - Place the system with no external load / MOSFET = OFF when powering on to get a valid ACS712 zero reading.
  - Set VOLTAGE_DIVIDER_RATIO to the actual resistor divider ratio (Vbat -> ADC).
  - Set ACS712_SENSITIVITY according to your module variant (typical: 0.185 V/A for 20A module,
    0.100 V/A for 5A module — check datasheet).
  - Tune KP_* gains carefully for your boost converter; gains are intentionally conservative.
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
const int PWM_FREQ = 50000;    // 50 kHz (typical for small boost converters)
const int PWM_RESOLUTION = 10; // bits (0..1023)
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;

// --- ADC / Sampling ---
const float ADC_MAX_VOLTAGE = 3.3f;  // Vref
const float ADC_MAX_READING = 4095.0f; // 12-bit
const int ADC_SAMPLES = 8; // average samples per reading
const int CALIBRATION_SAMPLES = 50; // number of averaged samples to compute ACS712 zero offset

// --- Voltage divider / current sensor calibration (USER MUST SET these for accuracy) ---
// Example: Voltage divider 100k:12k has ratio ~ (100k+12k)/12k = 9.333
const float VOLTAGE_DIVIDER_RATIO = 9.33f;    // Vbat = ADC_voltage * VOLTAGE_DIVIDER_RATIO
// ACS712 sensitivity (V per A). Typical values:
//   ACS712-05B ~ 0.100 V/A (for ±5A)
//   ACS712-20A ~ 0.100~0.185 V/A depending on exact variant (check module datasheet).
const float ACS712_SENSITIVITY = 0.185f;      // V per Amp (adjust to your module)

// Derived factors
const float ADC_V_PER_COUNT = (ADC_MAX_VOLTAGE / ADC_MAX_READING);
const float VOLTAGE_SENSE_FACTOR = ADC_V_PER_COUNT * VOLTAGE_DIVIDER_RATIO; // multiply raw ADC -> Vbat
const float CURRENT_RAW_TO_A = ADC_V_PER_COUNT / ACS712_SENSITIVITY; // multiply raw-diff -> A

// --- Charger State Machine ---
enum ChargerState_t { IDLE, CHARGING, PAUSED_CHECK_VOLTAGE, CHARGED_COMPLETE };
ChargerState_t chargerState = IDLE;

// --- Global Variables for readings & settings ---
float batteryVoltage = 0.0f;      // latest instantaneous computed voltage
float batteryVoltageFiltered = 0.0f; // smoothed for display/control
float chargingCurrent = 0.0f;     // instantaneous computed charging current (A)
float chargingCurrentFiltered = 0.0f; // smoothed
float desiredFinalVoltage = 29.0f; // from pot (V)
float desiredCurrentLimit = 1.0f;   // from pot (A)
int currentPwmDutyCycle = 0;       // 0..MAX_PWM_DUTY

// ACS712 calibration (raw ADC offset at 0 A)
int currentSensorOffsetRaw = 0;

// --- Timing ---
unsigned long lastChargeCheckTime = 0;
const unsigned long CHARGE_CHECK_INTERVAL_MS = 30000UL; // perform unloaded check every 30s
unsigned long pausedStartTime = 0;
const unsigned long PAUSE_SETTLE_MS = 500UL; // waiting time after PWM off to get unloaded reading

// --- Control parameters (tune carefully) ---
const float KP_CURRENT = 0.8f;       // proportional gain for current error (tune)
const float KP_BULK_VOLTAGE = 0.6f;  // proportional gain during bulk to approach bulk target
const float KP_FINAL_VOLTAGE = 0.5f; // proportional gain in CV mode

// Bulk target
const float BULK_CHARGE_VOLTAGE_TARGET = 30.0f;

// Safety
const float MAX_ALLOWED_CURRENT_MULTIPLIER = 2.5f; // if measured current > desiredCurrentLimit * this -> hard shutdown

// Sampling / smoothing alpha (0..1). Higher = faster response, lower = smoother.
const float FILTER_ALPHA_V = 0.25f;
const float FILTER_ALPHA_I = 0.30f;

// Function prototypes
void readSensorInputs();
void updateChargerState();
void updateOLED();
void setPwmDutyCycle(int dutyCycle);
int analogReadAveraged(int pin);
void calibrateCurrentSensor(bool verbose = true);
void enableOledOrWarn();


// ---------------------- Setup ----------------------
void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("=== ESP32 Charger (ACS712 auto-cal at startup) ===");

  // Configure pins
  pinMode(CHARGED_INDICATOR_PIN, OUTPUT);
  digitalWrite(CHARGED_INDICATOR_PIN, LOW);

  // Set up PWM (LEDC)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);
  setPwmDutyCycle(0); // ensure PWM off during init

  // ADC configuration for ESP32
  #if defined(ARDUINO_ARCH_ESP32)
    #if defined(analogReadResolution)
      analogReadResolution(12);
    #endif

    // Set attenuation to support up to ~3.3V at ADC input (adjust if needed)
    #if defined(ADC_ATTEN_DB_11)
      analogSetPinAttenuation(BAT_VOLTAGE_SENSE_PIN, ADC_ATTEN_DB_11);
      analogSetPinAttenuation(CURRENT_SENSE_AMP_PIN, ADC_ATTEN_DB_11);
      analogSetPinAttenuation(DESIRED_VOLTAGE_SET_PIN, ADC_ATTEN_DB_11);
      analogSetPinAttenuation(DESIRED_CURRENT_SET_PIN, ADC_ATTEN_DB_11);
    #elif defined(ADC_11db)
      analogSetPinAttenuation(BAT_VOLTAGE_SENSE_PIN, ADC_11db);
      analogSetPinAttenuation(CURRENT_SENSE_AMP_PIN, ADC_11db);
      analogSetPinAttenuation(DESIRED_VOLTAGE_SET_PIN, ADC_11db);
      analogSetPinAttenuation(DESIRED_CURRENT_SET_PIN, ADC_11db);
    #endif
  #endif

  // Initialize I2C and OLED (if present)
  Wire.begin(8, 7); // SDA, SCL — change if your wiring differs
  enableOledOrWarn();

  // Auto-calibrate ACS712 offset at startup: requires no current flowing.
  Serial.println("Auto-calibrating ACS712 zero offset (ensure no current flow)...");
  calibrateCurrentSensor(true);

  // Initialize read filters by doing one sensor read (so filters don't start at 0)
  readSensorInputs();
  batteryVoltageFiltered = batteryVoltage;
  chargingCurrentFiltered = chargingCurrent;

  lastChargeCheckTime = millis();
  Serial.println("Setup complete.");
}


// ---------------------- Loop ----------------------
void loop() {
  // Repeated tasks
  readSensorInputs();
  updateChargerState();
  updateOLED();

  // Allow runtime recalibration via serial command 'c'
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'c' || c == 'C') {
      Serial.println("Manual recalibration requested...");
      calibrateCurrentSensor(true);
    }
  }

  delay(100); // adjust for responsiveness vs CPU usage
}


// ---------------------- ADC helpers ----------------------
int analogReadAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    sum += analogRead(pin);
    // small gap helps ADC sample stability
    delayMicroseconds(400);
  }
  return (int)(sum / ADC_SAMPLES);
}


// ---------------------- ACS712 auto-calibration ----------------------
void calibrateCurrentSensor(bool verbose) {
  // Ensure PWM is off and MOSFET not conducting
  setPwmDutyCycle(0);
  delay(200); // allow sensor to settle near Vcc/2

  long sum = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    int sample = analogReadAveraged(CURRENT_SENSE_AMP_PIN);
    sum += sample;
    delay(6);
  }
  currentSensorOffsetRaw = (int)(sum / CALIBRATION_SAMPLES);

  if (verbose) {
    float offsetVolts = currentSensorOffsetRaw * ADC_V_PER_COUNT;
    Serial.print("ACS712 calibrated: raw offset = ");
    Serial.print(currentSensorOffsetRaw);
    Serial.print(" counts, ~");
    Serial.print(offsetVolts, 4);
    Serial.println(" V (should be ~Vcc/2)");
  }
}


// ---------------------- Read sensors and convert ----------------------
void readSensorInputs() {
  // Raw ADC averaged readings
  int rawBat = analogReadAveraged(BAT_VOLTAGE_SENSE_PIN);
  int rawI   = analogReadAveraged(CURRENT_SENSE_AMP_PIN);
  int rawSetV = analogReadAveraged(DESIRED_VOLTAGE_SET_PIN);
  int rawSetI = analogReadAveraged(DESIRED_CURRENT_SET_PIN);

  // Convert ADC -> physical values
  batteryVoltage = rawBat * VOLTAGE_SENSE_FACTOR;

  // Current: subtract zero offset (ACS712 Vcc/2)
  int correctedRaw = rawI - currentSensorOffsetRaw;
  float deltaV = correctedRaw * ADC_V_PER_COUNT;   // V above/below Vcc/2
  chargingCurrent = deltaV / ACS712_SENSITIVITY;   // A (signed)

  // Pot setpoints (example mapping: 0..3.3V -> 0..30V and 0..5A respectively)
  // If you prefer different mapping, adjust the factors below.
  // Compute on-the-fly factors for clarity
  float SET_V_POT_MAX_V = 30.0f; // pot maps to 0..30V
  float SET_I_POT_MAX_A = 5.0f;  // pot maps to 0..5A
  desiredFinalVoltage = rawSetV * (ADC_V_PER_COUNT) * (SET_V_POT_MAX_V / ADC_MAX_VOLTAGE);
  desiredCurrentLimit = rawSetI * (ADC_V_PER_COUNT) * (SET_I_POT_MAX_A / ADC_MAX_VOLTAGE);

  // Constrain setpoints to safe ranges (adjust to your battery spec)
  desiredFinalVoltage = constrain(desiredFinalVoltage, 26.0f, 30.0f);
  desiredCurrentLimit = constrain(desiredCurrentLimit, 0.05f, 5.0f);

  // Smoothing filters (exponential moving average)
  batteryVoltageFiltered = (batteryVoltageFiltered * (1.0f - FILTER_ALPHA_V)) + (batteryVoltage * FILTER_ALPHA_V);
  chargingCurrentFiltered = (chargingCurrentFiltered * (1.0f - FILTER_ALPHA_I)) + (chargingCurrent * FILTER_ALPHA_I);

  // Debug prints occasionally (every ~5s)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    lastDebug = millis();
    Serial.print("Vbat=");
    Serial.print(batteryVoltageFiltered, 3);
    Serial.print(" V, I=");
    Serial.print(chargingCurrentFiltered, 3);
    Serial.print(" A, setV=");
    Serial.print(desiredFinalVoltage, 2);
    Serial.print(" V, setI=");
    Serial.print(desiredCurrentLimit, 3);
    Serial.print(" A, offsetRaw=");
    Serial.println(currentSensorOffsetRaw);
  }
}


// ---------------------- PWM wrapper ----------------------
void setPwmDutyCycle(int dutyCycle) {
  int d = constrain(dutyCycle, 0, MAX_PWM_DUTY);
  currentPwmDutyCycle = d;
  ledcWrite(PWM_CHANNEL, currentPwmDutyCycle);
}


// ---------------------- Charger state machine ----------------------
void updateChargerState() {
  unsigned long now = millis();

  switch (chargerState) {
    case IDLE:
      // Keep PWM off in IDLE
      setPwmDutyCycle(0);

      // Auto-start if battery significantly below target
      if (batteryVoltageFiltered < (desiredFinalVoltage - 1.0f)) {
        chargerState = CHARGING;
        lastChargeCheckTime = now;
        digitalWrite(CHARGED_INDICATOR_PIN, LOW);
        Serial.println("State: IDLE -> CHARGING (auto-start)");
      }
      break;

    case CHARGING: {
      // Emergency: if measured current is absurdly high, kill PWM
      if (abs(chargingCurrentFiltered) > (desiredCurrentLimit * MAX_ALLOWED_CURRENT_MULTIPLIER + 0.1f)) {
        Serial.println("ERROR: measured current exceeds safe multiplier -> shutting PWM OFF!");
        setPwmDutyCycle(0);
        // To recover, operator must issue a recalibration or reset
        chargerState = IDLE;
        break;
      }

      // CC control (priority)
      if (chargingCurrentFiltered > desiredCurrentLimit) {
        // reduce duty proportional to current excess
        float err = (chargingCurrentFiltered - desiredCurrentLimit);
        // Map voltage-like error to duty: use fraction of max duty
        int delta = (int)(KP_CURRENT * err * (MAX_PWM_DUTY / max(0.5f, desiredCurrentLimit)));
        if (delta < 1) delta = 1;
        setPwmDutyCycle(currentPwmDutyCycle - delta);
        Serial.print("CC: overcurrent -> reduce PWM by "); Serial.print(delta); Serial.print(" to "); Serial.println(currentPwmDutyCycle);
      }
      // CV control
      else if (batteryVoltageFiltered >= desiredFinalVoltage) {
        float errV = (batteryVoltageFiltered - desiredFinalVoltage);
        // scale errV to PWM units relative to bulk target voltage
        int delta = (int)(KP_FINAL_VOLTAGE * errV * (MAX_PWM_DUTY / max(1.0f, BULK_CHARGE_VOLTAGE_TARGET)));
        if (delta < 1) delta = 1;
        setPwmDutyCycle(currentPwmDutyCycle - delta);
        Serial.print("CV: reduce by "); Serial.print(delta); Serial.print(" -> PWM "); Serial.println(currentPwmDutyCycle);
      }
      // Bulk charging
      else {
        float errV = (BULK_CHARGE_VOLTAGE_TARGET - batteryVoltageFiltered);
        int delta = (int)(KP_BULK_VOLTAGE * errV * (MAX_PWM_DUTY / max(1.0f, BULK_CHARGE_VOLTAGE_TARGET)));
        if (delta < 1) delta = 1;
        setPwmDutyCycle(currentPwmDutyCycle + delta);
        Serial.print("Bulk: increase by "); Serial.print(delta); Serial.print(" -> PWM "); Serial.println(currentPwmDutyCycle);
      }

      // Periodic paused check for accurate unloaded voltage measurement
      if (now - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
        Serial.println("Periodic check: pausing PWM to measure unloaded battery voltage...");
        setPwmDutyCycle(0);
        pausedStartTime = now;
        chargerState = PAUSED_CHECK_VOLTAGE;
      }
      break;
    }

    case PAUSED_CHECK_VOLTAGE:
      // Wait for voltage to settle after turning PWM off
      if (now - pausedStartTime < PAUSE_SETTLE_MS) {
        // do nothing; readSensorInputs runs each loop and updates batteryVoltageFiltered
        return;
      }

      // After settling, check if battery is at/above desired final voltage
      Serial.print("Paused voltage check: Vbat=");
      Serial.print(batteryVoltageFiltered, 3);
      Serial.print(" V vs set ");
      Serial.print(desiredFinalVoltage, 3);
      Serial.println(" V");

      if (batteryVoltageFiltered >= desiredFinalVoltage) {
        chargerState = CHARGED_COMPLETE;
        setPwmDutyCycle(0);
        digitalWrite(CHARGED_INDICATOR_PIN, HIGH);
        Serial.println("State: PAUSED_CHECK_VOLTAGE -> CHARGED_COMPLETE");
      } else {
        // resume charging
        chargerState = CHARGING;
        lastChargeCheckTime = now;
        Serial.println("State: PAUSED_CHECK_VOLTAGE -> CHARGING (voltage not reached)");
      }
      break;

    case CHARGED_COMPLETE:
      // keep PWM off and indicator lit
      setPwmDutyCycle(0);
      digitalWrite(CHARGED_INDICATOR_PIN, HIGH);

      // if battery drops, resume charging
      if (batteryVoltageFiltered <= (desiredFinalVoltage - 0.5f)) {
        chargerState = CHARGING;
        digitalWrite(CHARGED_INDICATOR_PIN, LOW);
        lastChargeCheckTime = now;
        Serial.println("State: CHARGED_COMPLETE -> CHARGING (voltage dropped)");
      }
      break;

    default:
      chargerState = IDLE;
      setPwmDutyCycle(0);
      break;
  }
}


// ---------------------- OLED UI ----------------------
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

  display.print("Bat V: ");
  display.print(batteryVoltageFiltered, 2);
  display.println(" V");

  display.print("Chg I: ");
  display.print(chargingCurrentFiltered, 3);
  display.println(" A");

  display.print("Set V: ");
  display.print(desiredFinalVoltage, 2);
  display.println(" V");

  display.print("Set I: ");
  display.print(desiredCurrentLimit, 2);
  display.println(" A");

  display.print("PWM: ");
  display.print(currentPwmDutyCycle);
  display.print(" / ");
  display.println(MAX_PWM_DUTY);

  display.print("Offset: ");
  display.println(currentSensorOffsetRaw);

  display.display();
}


// ---------------------- Utility: init OLED safely ----------------------
void enableOledOrWarn() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Warning: SSD1306 not found at 0x3C — continuing without OLED."));
    oledAvailable = false;
  } else {
    oledAvailable = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Charger Ready!");
    display.display();
    delay(400);
    display.clearDisplay();
    display.display();
  }
}
