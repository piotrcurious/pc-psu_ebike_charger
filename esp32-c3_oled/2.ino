#include <Arduino.h>
#include <Wire.h> // Required for I2C communication with the OLED display
#include <Adafruit_GFX.h> // Core graphics library
#include <Adafruit_SSD1306.h> // Specific library for SSD1306 OLED displays

// --- OLED Display Configuration ---
// The user specified a 72x40 OLED. Adafruit_SSD1306 typically supports 128x64 or 128x32.
// We'll use 128x64 as a common default, which should work for most small OLEDs,
// even if the visible area is smaller. You may need to adjust SCREEN_WIDTH/HEIGHT
// or the display address (0x3C or 0x3D) based on your specific 72x40 OLED module.
#define SCREEN_WIDTH 128 // OLED display width, in pixels (common for SSD1306)
#define SCREEN_HEIGHT 64 // OLED display height, in pixels (common for SSD1306)

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // I2C address for 128x64 OLED (common)

// Create the display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Pin Definitions ---
// GPIO10: PWM output for controlling the step-up (boost) converter.
// This pin will drive the gate of a MOSFET in your boost converter circuit.
const int PWM_OUT_PIN = 10;

// Analog Input Pins (ADC1 channels on ESP32-C3)
// GPIO0: Analog input for sensing the battery voltage.
// Connect this to a voltage divider that scales the battery voltage down to 0-3.3V.
const int BAT_VOLTAGE_SENSE_PIN = 0; // ADC1_CH0

// GPIO1: Analog input for reading the current from a current sensing amplifier.
// Connect this to the output of your current sensor (e.g., ACS712, INA219 output to ADC).
// Ensure the output voltage range of the amplifier is compatible with ESP32's 0-3.3V ADC.
const int CURRENT_SENSE_AMP_PIN = 1; // ADC1_CH1

// GPIO2: Analog input for setting the desired final charge voltage.
// Connect this to a potentiometer or another voltage source (0-3.3V) that sets the target.
const int DESIRED_VOLTAGE_SET_PIN = 2; // ADC1_CH2

// GPIO3: Analog input for setting the desired charging current limit.
// Connect this to a potentiometer or another voltage source (0-3.3V) that sets the limit.
const int DESIRED_CURRENT_SET_PIN = 3; // ADC1_CH3

// Digital Output Pin
// GPIO9: Indicator pin, set HIGH when charging is complete.
const int CHARGED_INDICATOR_PIN = 9;

// --- PWM Settings for Boost Converter Control ---
// LEDC (LED Controller) on ESP32 is used for PWM generation.
const int PWM_CHANNEL = 0; // Use LEDC channel 0
const int PWM_FREQ = 50000; // PWM frequency in Hz (50 kHz is common for boost converters)
const int PWM_RESOLUTION = 10; // 10-bit resolution (0 to 1023 duty cycle values)
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1; // Maximum duty cycle value (1023 for 10-bit)

// --- Calibration Constants ---
// These values are CRITICAL and MUST be calibrated for your specific hardware setup.
// Incorrect calibration will lead to inaccurate voltage/current readings and charging behavior.

// ESP32 ADC reference voltage (typically 3.3V for 12-bit ADC readings 0-4095)
const float ADC_MAX_VOLTAGE = 3.3;
const float ADC_MAX_READING = 4095.0; // 12-bit ADC resolution

// Example Calibration Factors (PLACEHOLDERS - YOU MUST CALIBRATE THESE!)
// To calibrate:
// 1. Measure a known voltage/current with a multimeter.
// 2. Read the raw ADC value from the ESP32.
// 3. Calculate the factor: Factor = (Known_Value / Raw_ADC_Value) * ADC_MAX_READING / ADC_MAX_VOLTAGE
//    Or, if you know your voltage divider ratio (R_total / R_low):
//    VOLTAGE_SENSE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * Voltage_Divider_Ratio;
//    Example: For a 30V max battery, if your divider outputs 3.0V at 30V, ratio is 10.
//    So, VOLTAGE_SENSE_FACTOR = (3.3 / 4095.0) * 10.0;
const float VOLTAGE_SENSE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * 9.33; // Placeholder: Example for a 100k/12k divider (ratio ~9.33) for max 30V
const float CURRENT_SENSE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * (1.0 / 0.185); // Placeholder: Example for ACS712-20A (185mV/A sensitivity)
const float SET_VOLTAGE_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * (30.0 / 3.3); // Placeholder: Maps 0-3.3V pot input to 0-30V target
const float SET_CURRENT_FACTOR = (ADC_MAX_VOLTAGE / ADC_MAX_READING) * (5.0 / 3.3); // Placeholder: Maps 0-3.3V pot input to 0-5A limit

// --- Charger State Machine ---
// Defines the different operational states of the battery charger.
enum ChargerState_t {
    IDLE,                 // Charger is off, waiting to start.
    CHARGING,             // Actively charging the battery (Constant Current or Constant Voltage).
    PAUSED_CHECK_VOLTAGE, // Charging paused temporarily to get an accurate battery voltage reading.
    CHARGED_COMPLETE      // Battery has reached the desired final voltage, charging is stopped.
};

ChargerState_t chargerState = IDLE; // Initial state of the charger

// --- Global Variables for Readings and Settings ---
float batteryVoltage = 0.0;     // Current measured battery voltage
float chargingCurrent = 0.0;    // Current measured charging current
float desiredFinalVoltage = 0.0; // Target voltage set by analog input (26-30V)
float desiredCurrentLimit = 0.0; // Max current limit set by analog input
int currentPwmDutyCycle = 0;    // The current PWM duty cycle applied to the boost converter

// --- Timers for Charging Logic ---
unsigned long lastChargeCheckTime = 0; // Stores the last time a voltage check was performed
const unsigned long CHARGE_CHECK_INTERVAL_MS = 30000; // 30 seconds (30,000 milliseconds)

// --- PWM Control Parameters (Proportional Gains) ---
// These gains are for a simple proportional control. They will likely require tuning
// for stable operation of your boost converter and desired responsiveness.
const float KP_CURRENT = 0.5;       // Proportional gain for current control (adjusts PWM based on current error)
const float KP_BULK_VOLTAGE = 1.0;  // Proportional gain for bulk voltage control (aiming for 30V)
const float KP_FINAL_VOLTAGE = 0.5; // Proportional gain for final constant voltage (CV) control

// --- Bulk Charging Target ---
// This is the voltage the charger will try to reach during the bulk charging phase,
// before the battery voltage gets close to the desiredFinalVoltage.
const float BULK_CHARGE_VOLTAGE_TARGET = 30.0; // User specified 30V for bulk charging

// --- Function Prototypes ---
void readSensorInputs();      // Reads all analog sensor values and converts them
void updateChargerState();    // Implements the state machine logic
void updateOLED();            // Updates the OLED display with current status
void setPwmDutyCycle(int dutyCycle); // Sets the PWM duty cycle for the boost converter

// --- Setup Function: Runs once when the ESP32 starts ---
void setup() {
    Serial.begin(115200); // Initialize serial communication for debugging
    Serial.println("ESP32-C3 Li-ion Charger Initializing...");

    // Configure the CHARGED_INDICATOR_PIN as an output
    pinMode(CHARGED_INDICATOR_PIN, OUTPUT);
    digitalWrite(CHARGED_INDICATOR_PIN, LOW); // Ensure indicator is off initially

    // Configure PWM for the boost converter
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Set up LEDC channel
    ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);         // Attach PWM channel to the output pin
    setPwmDutyCycle(0); // Start with PWM off to ensure no immediate output

    // Initialize OLED display
    // Wire.begin(SDA_PIN, SCL_PIN) sets the I2C pins.
    // For ESP32-C3, common I2C pins are GPIO8 (SDA) and GPIO7 (SCL).
    Wire.begin(8, 7); // SDA on GPIO8, SCL on GPIO7
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed. Check wiring and address!"));
        for (;;); // If OLED fails to initialize, halt execution.
    }

    // Display initial message on OLED
    display.display(); // Clear buffer and display initial content (usually blank)
    delay(2000); // Pause for 2 seconds
    display.clearDisplay(); // Clear the buffer
    display.setTextSize(1); // Set text size (1 is smallest)
    display.setTextColor(SSD1306_WHITE); // Set text color
    display.setCursor(0, 0); // Set cursor to top-left corner
    display.println("Charger Ready!");
    display.display(); // Show the message
    delay(1000); // Keep message on screen for 1 second
}

// --- Loop Function: Runs repeatedly after setup() ---
void loop() {
    readSensorInputs();   // Read all analog sensors
    updateChargerState(); // Execute the charger's state machine logic
    updateOLED();         // Update the OLED display with current status
    delay(100);           // Small delay to stabilize readings and update rates
                          // Adjust this delay as needed for responsiveness vs. stability.
}

// --- Function to Read and Convert Sensor Inputs ---
void readSensorInputs() {
    // Read raw 12-bit ADC values (0-4095)
    int rawBatVoltage = analogRead(BAT_VOLTAGE_SENSE_PIN);
    int rawCurrentSense = analogRead(CURRENT_SENSE_AMP_PIN);
    int rawDesiredVoltage = analogRead(DESIRED_VOLTAGE_SET_PIN);
    int rawDesiredCurrent = analogRead(DESIRED_CURRENT_SET_PIN);

    // Convert raw ADC values to real-world units (Volts and Amperes)
    batteryVoltage = rawBatVoltage * VOLTAGE_SENSE_FACTOR;
    chargingCurrent = rawCurrentSense * CURRENT_SENSE_FACTOR;

    // Convert desired settings from analog inputs
    desiredFinalVoltage = rawDesiredVoltage * SET_VOLTAGE_FACTOR;
    desiredCurrentLimit = rawDesiredCurrent * SET_CURRENT_FACTOR;

    // Constrain desired values to safe and specified operating ranges
    desiredFinalVoltage = constrain(desiredFinalVoltage, 26.0, 30.0); // Output voltage range 26V to 30V
    desiredCurrentLimit = constrain(desiredCurrentLimit, 0.1, 5.0);   // Example: Current limit from 0.1A to 5.0A
                                                                      // Adjust these limits based on your battery and charger capabilities.
}

// --- Function to Set PWM Duty Cycle ---
void setPwmDutyCycle(int dutyCycle) {
    // Ensure the duty cycle is within the valid range (0 to MAX_PWM_DUTY)
    currentPwmDutyCycle = constrain(dutyCycle, 0, MAX_PWM_DUTY);
    ledcWrite(PWM_CHANNEL, currentPwmDutyCycle); // Apply the duty cycle
}

// --- Charger State Machine Logic ---
void updateChargerState() {
    unsigned long currentTime = millis(); // Get current time for non-blocking delays

    switch (chargerState) {
        case IDLE:
            // Charger is in IDLE state.
            // Automatically start charging if battery voltage is significantly below the target.
            // You could also add a button press to start charging here.
            if (batteryVoltage < (desiredFinalVoltage - 1.0)) { // If battery is 1V below target, auto-start
                chargerState = CHARGING;
                Serial.println("State: IDLE -> CHARGING (Auto-start)");
                lastChargeCheckTime = currentTime; // Reset timer for the 30-second check
                digitalWrite(CHARGED_INDICATOR_PIN, LOW); // Ensure indicator is off when charging starts
            }
            setPwmDutyCycle(0); // Keep PWM off while idle
            break;

        case CHARGING:
            // Actively charging the battery using a CC/CV approach.
            // Priority 1: Constant Current (CC) control
            if (chargingCurrent > desiredCurrentLimit) {
                // If current exceeds the desired limit, reduce PWM to bring it down.
                // This ensures the battery is not over-current charged.
                setPwmDutyCycle(currentPwmDutyCycle - (int)(KP_CURRENT * (chargingCurrent - desiredCurrentLimit)));
                Serial.print("State: CHARGING (CC) - Current too high. PWM: "); Serial.println(currentPwmDutyCycle);
            }
            // Priority 2: Constant Voltage (CV) control when close to desired final voltage
            else if (batteryVoltage >= desiredFinalVoltage) {
                // If battery voltage has reached or exceeded the final target,
                // reduce PWM to maintain the desiredFinalVoltage.
                setPwmDutyCycle(currentPwmDutyCycle - (int)(KP_FINAL_VOLTAGE * (batteryVoltage - desiredFinalVoltage)));
                Serial.print("State: CHARGING (CV) - Voltage reached. PWM: "); Serial.println(currentPwmDutyCycle);
            }
            // Priority 3: Bulk Charging (aim for BULK_CHARGE_VOLTAGE_TARGET, e.g., 30V)
            else { // batteryVoltage < desiredFinalVoltage AND chargingCurrent <= desiredCurrentLimit
                // In the bulk charging phase, increase PWM to raise voltage towards the
                // BULK_CHARGE_VOLTAGE_TARGET (e.g., 30V), constrained by current limit.
                setPwmDutyCycle(currentPwmDutyCycle + (int)(KP_BULK_VOLTAGE * (BULK_CHARGE_VOLTAGE_TARGET - batteryVoltage)));
                Serial.print("State: CHARGING (Bulk) - Aiming for "); Serial.print(BULK_CHARGE_VOLTAGE_TARGET, 1); Serial.print("V. PWM: "); Serial.println(currentPwmDutyCycle);
            }

            // Check if 30 seconds have passed to perform a voltage check.
            if (currentTime - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
                chargerState = PAUSED_CHECK_VOLTAGE;
                Serial.println("State: CHARGING -> PAUSED_CHECK_VOLTAGE");
                setPwmDutyCycle(0); // Turn off PWM to get an accurate, unloaded battery voltage reading
                lastChargeCheckTime = currentTime; // Reset timer for the next 30s check if charging resumes
            }
            break;

        case PAUSED_CHECK_VOLTAGE:
            // Charger is paused to check the battery voltage accurately.
            setPwmDutyCycle(0); // Ensure PWM is off

            Serial.print("State: PAUSED_CHECK_VOLTAGE - Battery Voltage: "); Serial.println(batteryVoltage);

            if (batteryVoltage >= desiredFinalVoltage) {
                // Battery has reached or exceeded the desired final charging voltage.
                chargerState = CHARGED_COMPLETE;
                digitalWrite(CHARGED_INDICATOR_PIN, HIGH); // Set GPIO9 high to indicate charging complete
                Serial.println("State: PAUSED_CHECK_VOLTAGE -> CHARGED_COMPLETE");
            } else {
                // Battery has not yet reached the desired final voltage. Resume charging.
                chargerState = CHARGING;
                Serial.println("State: PAUSED_CHECK_VOLTAGE -> CHARGING (Voltage not reached)");
                // PWM will be turned back on by the logic in the CHARGING state.
            }
            break;

        case CHARGED_COMPLETE:
            // Battery is fully charged. Charger is off.
            setPwmDutyCycle(0); // Ensure PWM is off
            digitalWrite(CHARGED_INDICATOR_PIN, HIGH); // Keep GPIO9 high

            // Resume charging if the battery voltage drops significantly (0.5V below target).
            if (batteryVoltage <= (desiredFinalVoltage - 0.5)) {
                chargerState = CHARGING;
                digitalWrite(CHARGED_INDICATOR_PIN, LOW); // Turn off indicator as charging resumes
                Serial.println("State: CHARGED_COMPLETE -> CHARGING (Voltage dropped)");
                lastChargeCheckTime = currentTime; // Reset timer for the 30-second check
            }
            break;
    }
}

// --- Function to Update OLED Display ---
void updateOLED() {
    display.clearDisplay(); // Clear the display buffer

    // Display Charger State
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("State: ");
    switch (chargerState) {
        case IDLE: display.println("IDLE"); break;
        case CHARGING: display.println("CHARGING"); break;
        case PAUSED_CHECK_VOLTAGE: display.println("PAUSED"); break;
        case CHARGED_COMPLETE: display.println("CHARGED!"); break;
    }

    // Display Battery Voltage
    display.print("Bat V: ");
    display.print(batteryVoltage, 2); // Display with 2 decimal places
    display.println("V");

    // Display Charging Current
    display.print("Chg I: ");
    display.print(chargingCurrent, 2);
    display.println("A");

    // Display Desired Final Voltage Setting
    display.print("Set V: ");
    display.print(desiredFinalVoltage, 2);
    display.println("V");

    // Display Desired Current Limit Setting
    display.print("Set I: ");
    display.print(desiredCurrentLimit, 2);
    display.println("A");

    // Display Current PWM Duty Cycle
    display.print("PWM: ");
    display.println(currentPwmDutyCycle);

    display.display(); // Push the buffer content to the actual display
}
