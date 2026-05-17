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
const int UI_BUTTON_PIN = 4;
const int TEMP_SENSE_PIN = 5;
const int FAN_PWM_PIN = 6;
const int SDA_PIN = 8;
const int SCL_PIN = 7;

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
const float ADC_MAX_VOLTAGE = 3.3f;
const float ADC_MAX_READING = 4095.0f;
const float ADC_V_PER_COUNT = (ADC_MAX_VOLTAGE / ADC_MAX_READING);
const float VOLTAGE_SENSE_FACTOR = ADC_V_PER_COUNT * VOLTAGE_DIVIDER_RATIO;
const float CURRENT_RAW_TO_A = ADC_V_PER_COUNT / ACS712_SENSITIVITY;

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
#define GRAPH_BUFFER_SIZE 64
const unsigned long GRAPH_UPDATE_INTERVAL_MS = 60000UL;
const unsigned long UI_REFRESH_INTERVAL_MS = 200UL;

#endif
