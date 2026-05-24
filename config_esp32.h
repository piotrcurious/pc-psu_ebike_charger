#ifndef CONFIG_ESP32_H
#define CONFIG_ESP32_H

#include <Arduino.h>

// --- TFT Display Configuration (SPI) ---
#define TFT_WIDTH 320
#define TFT_HEIGHT 240

// SPI Pins for ESP32-WROOM
const int TFT_MOSI = 23;
const int TFT_SCLK = 18;
const int TFT_CS   = 5;
const int TFT_DC   = 2;
const int TFT_RST  = 4;

// --- Pin Definitions (GPIO numbers for classic ESP32) ---
const int PWM_OUT_PIN = 25;
const int BAT_VOLTAGE_SENSE_PIN = 34;
const int CURRENT_SENSE_AMP_PIN = 35;
const int DESIRED_VOLTAGE_SET_PIN = 36;
const int DESIRED_CURRENT_SET_PIN = 33;
const int ATX_PS_ON_PIN = 26;
const int UI_BUTTON_PIN = 27;
const int TEMP_SENSE_PIN = 39;
const int FAN_PWM_PIN = 14;
const int TFT_BL_PIN = 32;

// --- Signal Polarity ---
#define ATX_PSU_ON  LOW
#define ATX_PSU_OFF HIGH

// --- PSU Nominal Input ---
const float INPUT_VOLTAGE_NOMINAL = 12.0f;

// --- PWM (LEDC) settings ---
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 50000;
const int PWM_RESOLUTION = 10;
const int MAX_PWM_DUTY = (1 << PWM_RESOLUTION) - 1;

const int FAN_PWM_CHANNEL = 1;
const int FAN_PWM_FREQ = 25000;
const int FAN_PWM_RES = 8;
const int MAX_FAN_DUTY = (1 << FAN_PWM_RES) - 1;

// --- ADC / Sampling ---
const int ADC_SAMPLES = 16;
const int CALIBRATION_SAMPLES = 100;

// --- Calibration ---
const float VOLTAGE_DIVIDER_RATIO = 9.33f;
const float ACS712_SENSITIVITY = 0.185f;

// NTC Thermistor Parameters
const float NTC_R_SERIES = 10000.0f;
const float NTC_NOMINAL_R = 10000.0f;
const float NTC_NOMINAL_T = 25.0f;
const float NTC_BETA = 3950.0f;

// --- Timing and Safety Limits ---
const unsigned long CHARGE_CHECK_INTERVAL_MS = 30000UL;
const unsigned long PAUSE_SETTLE_MS = 1000UL;
const unsigned long PSU_RESTART_COOLDOWN_MS = 20000UL;
const unsigned long MAX_CHARGE_TIME_MS = 12 * 3600 * 1000UL;
const float MAX_CHARGE_AH_LIMIT = 20.0f;
const int WATCHDOG_TIMEOUT_S = 8;
const unsigned long STATS_SAVE_INTERVAL_MS = 600000UL;

// --- Control Strategy Parameters ---
const int PWM_STEP_UP = 2;
const int PWM_STEP_DOWN_FAST = 5;
const int PWM_STEP_DOWN_SLOW = 1;
const float VOLTAGE_DEADBAND = 0.05f;
const float CURRENT_DEADBAND = 0.05f;
const float MIN_BATTERY_VOLTAGE = 15.0f;
const float MAX_ALLOWED_CURRENT_MULTIPLIER = 2.0f;
const float MAX_ALLOWED_TEMP = 60.0f;
const float MIN_ALLOWED_TEMP = 0.0f;
const float SOFT_START_RAMP_A_PER_S = 0.1f;
const float FULL_CHARGE_CURRENT_THRESHOLD = 0.1f;

// Fan Thresholds
const float FAN_TEMP_MIN = 35.0f;
const float FAN_TEMP_MAX = 55.0f;

// Filters
const float FILTER_ALPHA_V = 0.2f;
const float FILTER_ALPHA_I = 0.2f;
const float FILTER_ALPHA_T = 0.1f;

// --- Logging and UI ---
#define GRAPH_BUFFER_SIZE 160
const unsigned long GRAPH_UPDATE_INTERVAL_MS = 60000UL;
const unsigned long UI_REFRESH_INTERVAL_MS = 100UL; // Faster for TFT

#endif
