# ESP32 E-Bike Charger Firmware

Professional-grade, modular firmware for a DIY e-bike battery charger. It controls a boost converter powered by a standard PC ATX power supply. This repository supports both the ESP32-C3 (OLED) and ESP32-WROOM (Color TFT) variants.

## Features

### 1. Robust Control
- **Incremental CC/CV**: Replaced unstable PID with a deterministic step-based control strategy.
- **Soft-Start**: Current ramps up gradually to prevent PSU tripping.
- **Unloaded Voltage Sensing**: Periodically pauses charging to measure the true battery EMF and estimate internal resistance.

### 2. PSU Management
- **ATX PS_ON Control**: Active-low logic to enable the 12V rail only when charging.
- **Anti-Cycle Protection**: Mandatory 20-second cooldown between PSU power cycles.
- **Health Monitoring**: Detects if the PSU rail is active and delivering current.

### 3. Safety Suite
- **Over-Temperature**: Thermal shutdown using NTC thermistor (Steinhart-Hart model).
- **Over-Current**: Fast shutdown on current spikes.
- **Capacity Limit**: Cutoff after 20Ah to prevent cell damage.
- **Timeout**: 12-hour maximum charge duration.
- **Disconnect Detection**: Detects sudden removal of the battery during high-power charging.
- **Watchdog**: ESP32 Task WDT ensures the system never hangs.

### 4. Advanced UI
- **6 screens**: Status, Live Telemetry, Live Graph, Session Summary, Lifetime Stats, and System Diagnostics.
- **Interactive**: Long-press button to reset sessions or re-calibrate the current sensor.
- **Auto-scaling Graph**: Real-time voltage history with target indicator.

### 5. Data Persistence
- **Lifetime Ah/Wh**: Tracks total energy delivered across reboots using Flash storage.
- **Wear Leveling**: Intelligent save logic minimizes Flash writes.

## Architecture

- `Charger.cpp/h`: Core state machine and charging logic.
- `UI.cpp/h`: SSD1306 OLED display management (ESP32-C3).
- `UI_TFT.cpp/h`: 320x240 Color TFT display management (ESP32-WROOM, LovyanGFX).
- `Storage.h`: Wrapper for ESP32 Preferences (NVS).
- `Filter.h`: Template Exponential Moving Average (EMA) filter for noise reduction.
- `config.h`: Hardware pinout for ESP32-C3.
- `config_esp32.h`: Hardware pinout for ESP32-WROOM.

## Testing

The project includes a comprehensive C++ mock Arduino environment.

### Run all tests:
```bash
./run_tests.sh
```

### Test Suites:
1. `test_iteration.cpp`: Full standard charging cycle simulation.
2. `test_safety.cpp`: Validation of all fault shutdown conditions.
3. `test_robustness.cpp`: Edge cases like sudden battery removal.
4. `test_unit.cpp`: Granular verification of math and logic for every function.
5. `test_advanced.cpp`: Long-term drift and ADC noise resilience tests.
6. `test_tft.cpp`: Logic verification for the color TFT interface.

## Hardware Setup

### Variant 1: ESP32-C3 (OLED)
- **MCU**: ESP32-C3 (GPIO 8/7 for I2C, 10 for PWM).
- **Display**: SSD1306 128x64 I2C.
- **Current Sensor**: ACS712 (or similar) on GPIO 1.
- **PSU**: Standard ATX (Green wire to GPIO 9).
- **Temp**: 10k NTC on GPIO 5.

### Variant 2: ESP32-WROOM (TFT)
- **MCU**: ESP32-WROOM-32.
- **Display**: 320x240 SPI TFT (ILI9341).
- **SPI Pins**: MOSI (23), SCLK (18), CS (5), DC (2), RST (4), BL (12).
- **PWM Output**: GPIO 25.
- **Sensing**: Vbat (34), Ichg (35), Vset (36), Iset (33), Temp (39).
- **PSU Control**: PS_ON (26), UI Button (27).
