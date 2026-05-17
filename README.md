# ESP32-C3 E-Bike Charger Firmware

Professional-grade, modular firmware for a DIY e-bike battery charger based on the ESP32-C3. It controls a boost converter powered by a standard PC ATX power supply.

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
- `UI.cpp/h`: OLED display management and button interaction.
- `Storage.h`: Wrapper for ESP32 Preferences (NVS).
- `Filter.h`: Template Exponential Moving Average (EMA) filter for noise reduction.
- `config.h`: Hardware pinout and safety thresholds.

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

## Hardware Setup

- **MCU**: ESP32-C3 (GPIO 8/7 for I2C, 10 for PWM).
- **Display**: SSD1306 128x64 I2C.
- **Current Sensor**: ACS712 (or similar) on GPIO 1.
- **PSU**: Standard ATX (Green wire to GPIO 9).
- **Temp**: 10k NTC on GPIO 5.
