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
bool oledAvailable = false; // fallback if OLED init fails

// --- Pin Definitions ---
const int PWM_OUT_PIN = 10;                // PWM output for boost converter
const int BAT_VOLTAGE_SENSE_PIN = 0;       // ADC input for battery voltage (GPIO number)
const int CURRENT_SENSE_AMP_PIN = 1;       // ADC input for current sense
const int DESIRED_VOLTAGE_SET_PIN = 2;     // ADC input for setpoint pot (voltage)
const int DESIRED_CURRENT_SET_PIN = 3;     // ADC input for setpoint pot (current)
const int CHARGED_INDICATOR_PIN = 9;       // Digital output indicator (LED)

// --- PWM (LEDC) settings ---
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;    // 50 kHz
const int PWM_RESOLUTION = 10; // bits
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;

// --- ADC / Calibration Constants ---
// NOTE: You MUST calibrate these for your hardware. The placeholders are examples only.
const float ADC_MAX_VOLTAGE = 3.3f;
const float ADC_MAX_READING = 4095.0f; // 12-bit ADC

// Placeholder factors: replace with measured/calculated values for your divider/sensor.
const float VOLTAGE_SENSE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * 9.33f; // example for ~9.33 divider
const float CURRENT_SENSE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * (1.0f / 0.185f); // example for ACS712-20A
const float SET_VOLTAGE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * (30.0f / 3.3f); // map 0-3.3V pot to 0-30V
const float SET_CURRENT_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * (5.0f / 3.3f);   // map 0-3.3V pot to 0-5A

// --- Charger State Machine ---
enum ChargerState_t {
  IDLE,
  CHARGING,
  PAUSED_CHECK_VOLTAGE,
  CHARGED_COMPLETE
};
ChargerState_t chargerState = IDLE;

// --- Globals ---
float batteryVoltage = 0.0f;
float chargingCurrent = 0.0f;
float desiredFinalVoltage = 0.0f;
float desiredCurrentLimit = 0.0f;
int currentPwmDutyCycle = 0;

// --- Timers & intervals ---
unsigned long lastChargeCheckTime = 0;
const unsigned long CHARGE_CHECK_INTERVAL_MS = 30000UL; // 30 seconds
unsigned long pausedStartTime = 0;
const unsigned long PAUSE_SETTLE_MS = 500UL; // Wait 500ms after PWM off to read unloaded voltage

// --- Sampling ---
const int ADC_SAMPLES = 8; // average 8 samples

// --- Control gains (tune carefully) ---
const float KP_CURRENT = 0.5f;
const float KP_BULK_VOLTAGE = 1.0f;
const float KP_FINAL_VOLTAGE = 0.5f;

// --- Bulk charge target ---
const float BULK_CHARGE_VOLTAGE_TARGET = 30.0f;

// --- Forward declarations ---
void readSensorInputs();
void updateChargerState();
void updateOLED();
void setPwmDutyCycle(int dutyCycle);
int analogReadAveraged(int pin);

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(5);
  Serial.println();
  Serial.println("ESP32-C3 (or compatible) Charger - initializing");

  pinMode(CHARGED_INDICATOR_PIN, OUTPUT);
  digitalWrite(CHARGED_INDICATOR_PIN, LOW);

  // Setup PWM (LEDC)
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);
  setPwmDutyCycle(0);

  // ADC configuration for ESP32
  #if defined(ARDUINO_ARCH_ESP32)
    // Ensure 12-bit resolution if available
    #if defined(analogReadResolution)
      analogReadResolution(12);
    #endif
    // Set attenuation to support up to ~3.3V reading (adjust if your divider yields less)
    #if defined(ADC_11db)
      // Set attenuation for pins used (these functions exist on ESP32 cores)
      analogSetPinAttenuation(BAT_VOLTAGE_SENSE_PIN, ADC_11db);
      analogSetPinAttenuation(CURRENT_SENSE_AMP_PIN, ADC_11db);
      analogSetPinAttenuation(DESIRED_VOLTAGE_SET_PIN, ADC_11db);
      analogSetPinAttenuation(DESIRED_CURRENT_SET_PIN, ADC_11db);
    #endif
  #endif

  // Initialize I2C for OLED
  // On ESP32-C3 common I2C pins might be 8 (SDA) and 7 (SCL) as in your original sketch.
  // Change if your wiring differs.
  Wire.begin(8, 7);

  // Initialize OLED, but don't lock up if it fails. Run without OLED as fallback.
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Warning: SSD1306 not found at 0x3C (OLED disabled). Check wiring or address."));
    oledAvailable = false;
  } else {
    oledAvailable = true;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Charger Ready!");
    display.display();
    delay(1000);
    display.clearDisplay();
    display.display();
  }

  // Initialize timings
  lastChargeCheckTime = millis();
  Serial.println("Initialization complete.");
}

// ------------------- Loop -------------------
void loop() {
  readSensorInputs();
  updateChargerState();
  updateOLED();
  delay(100); // small delay to limit update rate
}

// ------------------- Read sensors (with averaging) -------------------
int analogReadAveraged(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    sum += analogRead(pin);
    delayMicroseconds(500); // tiny gap between samples
  }
  return (int)(sum / ADC_SAMPLES);
}

void readSensorInputs() {
  // Read averaged ADC values
  int rawBatVoltage = analogReadAveraged(BAT_VOLTAGE_SENSE_PIN);
  int rawCurrentSense = analogReadAveraged(CURRENT_SENSE_AMP_PIN);
  int rawDesiredVoltage = analogReadAveraged(DESIRED_VOLTAGE_SET_PIN);
  int rawDesiredCurrent = analogReadAveraged(DESIRED_CURRENT_SET_PIN);

  // Convert to physical units (user must calibrate these factors)
  batteryVoltage = rawBatVoltage * VOLTAGE_SENSE_FACTOR;
  chargingCurrent = rawCurrentSense * CURRENT_SENSE_FACTOR;
  desiredFinalVoltage = rawDesiredVoltage * SET_VOLTAGE_FACTOR;
  desiredCurrentLimit = rawDesiredCurrent * SET_CURRENT_FACTOR;

  // Constrain setpoints to safe ranges
  desiredFinalVoltage = constrain(desiredFinalVoltage, 26.0f, 30.0f);
  desiredCurrentLimit = constrain(desiredCurrentLimit, 0.1f, 5.0f);
}

// ------------------- PWM -------------------
void setPwmDutyCycle(int dutyCycle) {
  int d = constrain(dutyCycle, 0, MAX_PWM_DUTY);
  currentPwmDutyCycle = d;
  ledcWrite(PWM_CHANNEL, currentPwmDutyCycle);
}

// ------------------- State machine -------------------
void updateChargerState() {
  unsigned long currentTime = millis();

  switch (chargerState) {
    case IDLE:
      // Auto-start if battery is at least 1.0V below desired voltage
      if (batteryVoltage < (desiredFinalVoltage - 1.0f)) {
        chargerState = CHARGING;
        Serial.println("Transition: IDLE -> CHARGING (auto-start)");
        lastChargeCheckTime = currentTime;
        digitalWrite(CHARGED_INDICATOR_PIN, LOW);
      }
      // Keep PWM off while idle
      setPwmDutyCycle(0);
      break;

    case CHARGING:
      // Constant current control (priority)
      if (chargingCurrent > desiredCurrentLimit) {
        float err = (chargingCurrent - desiredCurrentLimit);
        int delta = (int)(KP_CURRENT * err);
        setPwmDutyCycle(currentPwmDutyCycle - delta);
        Serial.print("CC: current "); Serial.print(chargingCurrent, 2); Serial.print("A > limit ");
        Serial.print(desiredCurrentLimit, 2); Serial.print("A -> PWM: "); Serial.println(currentPwmDutyCycle);
      }
      // Constant voltage control when at/above desiredVoltage (CV)
      else if (batteryVoltage >= desiredFinalVoltage) {
        float errV = (batteryVoltage - desiredFinalVoltage);
        int delta = (int)(KP_FINAL_VOLTAGE * errV);
        setPwmDutyCycle(currentPwmDutyCycle - delta);
        Serial.print("CV: batV "); Serial.print(batteryVoltage, 2); Serial.print("V >= set ");
        Serial.print(desiredFinalVoltage, 2); Serial.print("V -> PWM: "); Serial.println(currentPwmDutyCycle);
      }
      // Bulk charging: push towards bulk target but remain mindful of current limit
      else {
        float errV = (BULK_CHARGE_VOLTAGE_TARGET - batteryVoltage);
        int delta = (int)(KP_BULK_VOLTAGE * errV);
        setPwmDutyCycle(currentPwmDutyCycle + delta);
        Serial.print("Bulk: target "); Serial.print(BULK_CHARGE_VOLTAGE_TARGET, 2);
        Serial.print("V, batV "); Serial.print(batteryVoltage, 2);
        Serial.print("V -> PWM: "); Serial.println(currentPwmDutyCycle);
      }

      // Periodic check: pause to get accurate unloaded voltage reading
      if (currentTime - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
        chargerState = PAUSED_CHECK_VOLTAGE;
        setPwmDutyCycle(0); // disable PWM to get unloaded reading
        pausedStartTime = currentTime;
        Serial.println("Transition: CHARGING -> PAUSED_CHECK_VOLTAGE (PWM off to measure unloaded V)");
      }
      break;

    case PAUSED_CHECK_VOLTAGE:
      // Wait a small settle period to allow voltage to stabilize with PWM off.
      if (currentTime - pausedStartTime < PAUSE_SETTLE_MS) {
        // Do nothing (we rely on readSensorInputs() in loop to update readings)
        return;
      }

      // After settling, we expect readSensorInputs() to have updated batteryVoltage
      Serial.print("Paused check - unloaded battery voltage: ");
      Serial.print(batteryVoltage, 3); Serial.println(" V");

      if (batteryVoltage >= desiredFinalVoltage) {
        chargerState = CHARGED_COMPLETE;
        digitalWrite(CHARGED_INDICATOR_PIN, HIGH);
        Serial.println("Transition: PAUSED_CHECK_VOLTAGE -> CHARGED_COMPLETE");
      } else {
        chargerState = CHARGING;
        lastChargeCheckTime = currentTime; // reset check timer when resuming
        Serial.println("Transition: PAUSED_CHECK_VOLTAGE -> CHARGING (voltage not reached)");
      }
      break;

    case CHARGED_COMPLETE:
      setPwmDutyCycle(0);
      digitalWrite(CHARGED_INDICATOR_PIN, HIGH);

      // If the battery has fallen sufficiently below the desired voltage, resume charging.
      if (batteryVoltage <= (desiredFinalVoltage - 0.5f)) {
        chargerState = CHARGING;
        digitalWrite(CHARGED_INDICATOR_PIN, LOW);
        lastChargeCheckTime = currentTime;
        Serial.println("Transition: CHARGED_COMPLETE -> CHARGING (voltage dropped)");
      }
      break;

    default:
      // Safety: fallback to IDLE
      chargerState = IDLE;
      setPwmDutyCycle(0);
      break;
  }
}

// ------------------- OLED UI -------------------
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
  display.print(batteryVoltage, 2);
  display.println(" V");

  display.print("Chg I: ");
  display.print(chargingCurrent, 2);
  display.println(" A");

  display.print("Set V: ");
  display.print(desiredFinalVoltage, 2);
  display.println(" V");

  display.print("Set I: ");
  display.print(desiredCurrentLimit, 2);
  display.println(" A");

  display.print("PWM: ");
  display.println(currentPwmDutyCycle);

  display.display();
}
