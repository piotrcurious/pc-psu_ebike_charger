#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- OLED Display Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// --- Pin Definitions (GPIO numbers) ---
const int PWM_OUT_PIN = 10;
const int BAT_VOLTAGE_SENSE_PIN = 0;
const int CURRENT_SENSE_AMP_PIN = 1;
const int DESIRED_VOLTAGE_SET_PIN = 2;
const int DESIRED_CURRENT_SET_PIN = 3;
const int ATX_PS_ON_PIN = 9;
const int UI_BUTTON_PIN = 4; // Use GPIO 4 for UI interaction

// --- Signal Polarity ---
#define ATX_PSU_ON  LOW
#define ATX_PSU_OFF HIGH

// --- PWM (LEDC) settings ---
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 10;
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;

// --- ADC / Sampling ---
const int ADC_SAMPLES = 16;
const int CALIBRATION_SAMPLES = 100;

// --- Calibration ---
const float VOLTAGE_DIVIDER_RATIO = 9.33f;
const float ACS712_SENSITIVITY = 0.185f;
const float ADC_MAX_VOLTAGE = 3.3f;
const float ADC_MAX_READING = 4095.0f;
const float ADC_V_PER_COUNT = (ADC_MAX_VOLTAGE / ADC_MAX_READING);
const float VOLTAGE_SENSE_FACTOR = ADC_V_PER_COUNT * VOLTAGE_DIVIDER_RATIO;
const float CURRENT_RAW_TO_A = ADC_V_PER_COUNT / ACS712_SENSITIVITY;

// --- Timing ---
const unsigned long CHARGE_CHECK_INTERVAL_MS = 30000UL;
const unsigned long PAUSE_SETTLE_MS = 1000UL;
const unsigned long PSU_RESTART_COOLDOWN_MS = 10000UL;
const unsigned long MAX_CHARGE_TIME_MS = 12 * 3600 * 1000UL;

// --- Control Strategy Parameters ---
const int PWM_STEP_UP = 2;
const int PWM_STEP_DOWN_FAST = 5;
const int PWM_STEP_DOWN_SLOW = 1;
const float VOLTAGE_DEADBAND = 0.05f;
const float CURRENT_DEADBAND = 0.05f;
const float MIN_BATTERY_VOLTAGE = 15.0f;
const float MAX_ALLOWED_CURRENT_MULTIPLIER = 2.0f;
const float SOFT_START_RAMP_A_PER_S = 0.1f;
const float FULL_CHARGE_CURRENT_THRESHOLD = 0.1f;

// Filters
const float FILTER_ALPHA_V = 0.2f;
const float FILTER_ALPHA_I = 0.2f;

// --- Logging and UI ---
#define GRAPH_BUFFER_SIZE 64
const unsigned long GRAPH_UPDATE_INTERVAL_MS = 60000UL; // Log point every minute

#endif
