#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include "mock_arduino/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "mock_arduino/Wire/Wire.h"
#include "mock_arduino/Preferences.h"

TwoWire WireStub;
Adafruit_SSD1306 display(128, 64, &WireStub, -1);

#include "../Charger.cpp"
#include "../UI.cpp"

extern void setAnalogRead(uint8_t pin, int value);
extern void queueAnalogRead(uint8_t pin, int value);
extern void setDigitalRead(uint8_t pin, uint8_t val);
extern void advance_millis(unsigned long ms);

void test_ema_filter() {
    std::cout << "Testing EMAFilter..." << std::endl;
    EMAFilter<float> filter(0.5);
    assert(filter.value() == 0);

    filter.update(10.0);
    // First update initializes it to the value
    assert(filter.value() == 10.0);

    filter.update(20.0);
    // 0.5 * 20 + 0.5 * 10 = 15.0
    assert(filter.value() == 15.0);

    filter.reset();
    // After reset, value() might still be old but _initialized is false.
    // In our implementation, value() returns (T)_filteredValue which was 15.0
    // But next update will overwrite it.
    std::cout << "EMAFilter test PASSED" << std::endl;
}

void test_calculate_temp() {
    std::cout << "Testing Temperature Calculation..." << std::endl;
    Charger charger;
    // We test via readSensors + temp()
    // Nominal 25C case:
    // V = 1.65 (half of 3.3), R_ntc = R_series = 10k.
    setAnalogRead(TEMP_SENSE_PIN, 2048); // raw=2048 -> ~1.65V
    charger.update(0.1);
    float t = charger.temp();
    std::cout << "Temp at 2048: " << t << " C" << std::endl;
    assert(t > 24.0 && t < 26.0);

    // High temp case: low resistance, low voltage (NTC where R decreases with T)
    // Wait, let's look at calculateTemp again.
    // voltage = rawADC * ADC_V_PER_COUNT;
    // resistance = NTC_R_SERIES * (voltage / (3.3f - voltage));
    // steinhart = resistance / NTC_NOMINAL_R;
    // steinhart = log(steinhart);
    // This formula implies R_ntc is the upper resistor in the divider if V increases with R_ntc?
    // No, V = Vcc * R_series / (R_ntc + R_series) -> higher R_ntc = lower V.
    // If V = Vcc * R_ntc / (R_ntc + R_series) -> higher R_ntc = higher V.
    // The formula resistance = NTC_R_SERIES * (voltage / (3.3f - voltage))
    // is R_ntc = R_series * (V / (Vcc - V)), which matches V = Vcc * R_ntc / (R_ntc + R_series).
    // So HIGHER voltage (raw ADC) means HIGHER resistance, which means LOWER temperature for NTC.
    // So to get HIGH temp, we need LOW voltage (LOW raw ADC).

    setAnalogRead(TEMP_SENSE_PIN, 500);
    for(int i=0; i<100; i++) charger.update(0.1);
    t = charger.temp();
    std::cout << "Temp at 500: " << t << " C" << std::endl;
    // At 500/4095 * 3.3 = 0.4V. R = 10k * (0.4 / 2.9) = 1.37k.
    // R/R_nom = 0.137. ln(0.137) = -1.98. /3950 = -0.0005.
    // 1/T = 1/298 - 0.0005 = 0.00335 - 0.0005 = 0.00285. T = 350K = 77C.
    assert(t > 50.0);

    // Low temp case: high resistance, high voltage
    setAnalogRead(TEMP_SENSE_PIN, 3800);
    for(int i=0; i<100; i++) charger.update(0.1);
    t = charger.temp();
    std::cout << "Temp at 3800: " << t << " C" << std::endl;
    assert(t < 10.0);

    std::cout << "Temperature Calculation test PASSED" << std::endl;
}

void test_psu_cooldown() {
    std::cout << "Testing PSU Cooldown..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.setup();

    // Trigger charging
    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING && charger.isPsuOn()) break;
    }
    assert(charger.isPsuOn());

    // Force error to turn PSU OFF
    setAnalogRead(TEMP_SENSE_PIN, 100);
    for(int i=0; i<100; i++) {
        charger.update(0.1);
        if (charger.state() == ERROR_STATE && !charger.isPsuOn()) break;
    }
    assert(!charger.isPsuOn());

    // Clear error (in a real scenario we'd need a reset, but here let's see if IDLE tries to turn it on)
    charger.reset();
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));

    // Try to restart immediately
    charger.update(0.1);
    // PSU should still be OFF because of cooldown
    assert(!charger.isPsuOn());

    // Wait 5s
    advance_millis(5000);
    charger.update(0.1);
    assert(!charger.isPsuOn());

    // Wait another 16s (total 21s)
    advance_millis(16000);
    charger.update(0.1);
    assert(charger.isPsuOn());

    std::cout << "PSU Cooldown test PASSED" << std::endl;
}

void test_integrators() {
    std::cout << "Testing Integrators..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048); // Zero current for calibration
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3000);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 3000); // High current limit
    charger.setup();

    // Get to charging
    for(int i=0; i<400; i++) {
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.0 / CURRENT_RAW_TO_A));
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // Charge for 1 hour (3600s)
    // We'll do it in steps of 1s
    for(int i=0; i<3600; i++) {
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.0 / CURRENT_RAW_TO_A));
        advance_millis(1000);
        charger.update(1.0);
        if (charger.state() == PAUSED_CHECK_VOLTAGE) {
            advance_millis(PAUSE_SETTLE_MS + 100);
            charger.update(0.1);
        }
    }

    // 2A for 1 hour should be 2Ah
    std::cout << "Ah after 1 hour at 2A: " << charger.ah() << std::endl;
    assert(std::abs(charger.ah() - 2.0) < 0.05);

    // Wh: 2A * 24V * 1h = 48Wh
    std::cout << "Wh after 1 hour at 2A/24V: " << charger.wh() << std::endl;
    assert(std::abs(charger.wh() - 48.0) < 1.0);

    std::cout << "Integrators test PASSED" << std::endl;
}

void test_ui_button() {
    std::cout << "Testing UI Button Debounce and Long-press..." << std::endl;
    Charger charger;
    UI ui(display, charger);
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    ui.setup();

    // Short press
    setDigitalRead(UI_BUTTON_PIN, LOW);
    advance_millis(100);
    ui.update(); // Button pressed
    advance_millis(100);
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    advance_millis(100);
    ui.update(); // Button released -> Screen change

    // Assuming screen 0 -> 1
    // We can't easily check private screen index without making it public or using a friend class
    // But we can check if it compiles and runs without crashing for now.

    // Long press
    setDigitalRead(UI_BUTTON_PIN, LOW);
    advance_millis(2500);
    ui.update();
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    advance_millis(100);
    ui.update();

    std::cout << "UI Button test PASSED (logic executed)" << std::endl;
}

void test_analog_read_averaged() {
    std::cout << "Testing analogReadAveraged..." << std::endl;
    Charger charger;
    // analogReadMilliVoltsAveraged takes ADC_SAMPLES (16) samples
    for (int i = 0; i < 16; i++) {
        queueAnalogRead(BAT_VOLTAGE_SENSE_PIN, 1000); // 1000/4095 * 3300 = 805mV
    }
    // setup() calls calibrateCurrentSensor which reads 100 samples.
    // We need to provide those samples or let it read from default value
    for (int i = 0; i < 100; i++) {
        queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    }

    // setup() also calls readSensors() once
    for (int i = 0; i < 16; i++) {
        queueAnalogRead(BAT_VOLTAGE_SENSE_PIN, 1000 + i);
    }
    for (int i = 0; i < 16; i++) queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(TEMP_SENSE_PIN, 2048);

    charger.setup();

    // Now provide samples for the update() call which we want to test
    for (int i = 0; i < 16; i++) {
        queueAnalogRead(BAT_VOLTAGE_SENSE_PIN, 1000 + i);
    }
    for (int i = 0; i < 16; i++) queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(TEMP_SENSE_PIN, 2048);

    charger.update(0.1);

    float expectedV = (1000.0f * 3300.0f / 4095.0f) * 0.001f * VOLTAGE_DIVIDER_RATIO;
    std::cout << "Expected V: " << expectedV << " actual: " << charger.vBat() << std::endl;
    assert(std::abs(charger.vBat() - expectedV) < 0.5); // Wider tolerance for float vs uint32
    std::cout << "analogReadAveraged test PASSED" << std::endl;
}

void test_storage() {
    std::cout << "Testing ChargerStorage..." << std::endl;
    ChargerStorage storage;
    storage.begin();
    storage.save(1.23f, 45.6f);
    assert(storage.lifetimeAh() == 1.23f);
    assert(storage.lifetimeWh() == 45.6f);

    ChargerStorage storage2;
    storage2.begin();
    // Should reload from map
    assert(std::abs(storage2.lifetimeAh() - 1.23f) < 0.001);
    assert(std::abs(storage2.lifetimeWh() - 45.6f) < 0.001);
    std::cout << "ChargerStorage test PASSED" << std::endl;
}

void test_calibration() {
    std::cout << "Testing Current Sensor Calibration..." << std::endl;
    Charger charger;
    // Calibration takes CALIBRATION_SAMPLES (100)
    for (int i = 0; i < 100; i++) {
        queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048); // ~1650mV
    }
    charger.calibrateCurrentSensor();
    assert(std::abs((int)charger.currentOffsetRaw() - 1650) < 5);

    // Now test if current is 0 when reading 2100
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2100);
    charger.update(0.1);
    assert(std::abs(charger.iChg()) < 0.001);
    std::cout << "Current Calibration test PASSED" << std::endl;
}

void test_fan_control() {
    std::cout << "Testing Fan Control..." << std::endl;
    Charger charger;
    // FAN_TEMP_MIN = 35.0, FAN_TEMP_MAX = 55.0
    // Below MIN
    setAnalogRead(TEMP_SENSE_PIN, 2048); // ~25C
    charger.update(0.1);
    assert(charger.fanDuty() == 0);

    // Above MAX
    setAnalogRead(TEMP_SENSE_PIN, 500); // ~77C
    for(int i=0; i<100; i++) charger.update(0.1);
    assert(charger.fanDuty() == MAX_FAN_DUTY);

    // Mid point
    // We want ~45C.
    // 1/T = 1/298 + ln(R/Rnom)/BETA
    // 1/318 = 0.003144
    // 0.003144 - 0.003355 = -0.000211
    // -0.000211 * 3950 = -0.833 = ln(R/Rnom)
    // R/Rnom = 0.43 -> R = 4.3k
    // V = 3.3 * 4.3 / 14.3 = 1.0V
    // Raw = 1.0 / 3.3 * 4095 = 1240
    setAnalogRead(TEMP_SENSE_PIN, 1240);
    for(int i=0; i<100; i++) charger.update(0.1);
    std::cout << "Fan duty at 45C: " << charger.fanDuty() << std::endl;
    assert(charger.fanDuty() > 0 && charger.fanDuty() < MAX_FAN_DUTY);

    std::cout << "Fan Control test PASSED" << std::endl;
}

void test_setpoints() {
    std::cout << "Testing Target Setpoints..." << std::endl;
    // targetVoltage: constrain((float)mvSetV * 0.001f * (30.0f / 3.3f), 26.0f, 30.0f)
    // currentLimit: constrain((float)mvSetI * 0.001f * (5.0f / 3.3f), 0.1f, 5.0f)
    Charger charger;

    // Set to 4095 (3.3V) -> Should yield 30V and 5A
    for(int i=0; i<32; i++) {
        setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 4095);
        setAnalogRead(DESIRED_CURRENT_SET_PIN, 4095);
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, 2048);
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
        setAnalogRead(TEMP_SENSE_PIN, 2048);
        charger.update(0.1);
    }
    // With SF check at 3250mV, 3300mV will cause error
    if (charger.state() == ERROR_STATE) {
        std::cout << "Target Setpoints errored due to sensor fault (expected at 3300mV)" << std::endl;
        charger.reset();
    }

    // Set to 3000 (2.42V)
    for(int i=0; i<32; i++) {
        setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3000);
        setAnalogRead(DESIRED_CURRENT_SET_PIN, 3000);
        charger.update(0.1);
    }
    // 2.42V * (30/3.3) = 22V -> constrained to 26V
    // 2.42V * (5/3.3) = 3.66A
    std::cout << "Target V: " << charger.targetVoltage() << " Target I: " << charger.currentLimit() << std::endl;
    assert(charger.targetVoltage() >= 26.0);
    assert(std::abs(charger.currentLimit() - 3.66) < 0.1);

    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 0);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 0);
    for(int i=0; i<32; i++) charger.update(0.1);
    assert(std::abs(charger.targetVoltage() - 26.0) < 0.1);
    assert(std::abs(charger.currentLimit() - 0.1) < 0.1);

    std::cout << "Target Setpoints test PASSED" << std::endl;
}

void test_reset_logic() {
    std::cout << "Testing Reset Logic..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.setup();

    // Ensure setpoints are reasonable
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048); // ~28V
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 2048); // ~2.5A

    // Wait for cooldown and transition to charging
    for(int i=0; i<500; i++) {
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(1.0 / CURRENT_RAW_TO_A));
        setAnalogRead(TEMP_SENSE_PIN, 2048);
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING && charger.ah() > 0) break;
    }
    if (charger.state() != CHARGING) {
        std::cout << "Reset Logic FAILED to enter CHARGING. State: " << (int)charger.state() << " PSU: " << (int)charger.isPsuOn() << " Vbat: " << charger.vBat() << std::endl;
    }
    assert(charger.state() == CHARGING);
    assert(charger.ah() > 0);

    charger.resetSession();
    assert(charger.ah() == 0);
    assert(charger.state() == CHARGING); // state remains

    // Test full reset
    charger.reset();
    assert(charger.state() == IDLE);
    assert(charger.ah() == 0);

    std::cout << "Reset Logic test PASSED" << std::endl;
}

void test_ui_render() {
    std::cout << "Testing UI Rendering and Content..." << std::endl;
    Charger charger;
    UI ui(display, charger);
    ui.setup();

    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.update(0.1);

    // Screen 0: Status
    // UI refreshes every UI_REFRESH_INTERVAL_MS (200ms)
    advance_millis(250);
    ui.update();
    std::string buf = display.getBuffer();
    assert(buf.find("IDLE") != std::string::npos);
    assert(buf.find("24.0V") != std::string::npos);

    // Switch to screen 1: Live
    setDigitalRead(UI_BUTTON_PIN, LOW); advance_millis(100); ui.update();
    setDigitalRead(UI_BUTTON_PIN, HIGH); advance_millis(100); ui.update();
    buf = display.getBuffer();
    assert(buf.find("24.00 V") != std::string::npos);

    // Test error screen content
    setAnalogRead(TEMP_SENSE_PIN, 100);
    for(int i=0; i<100; i++) charger.update(0.1);
    assert(charger.state() == ERROR_STATE);
    advance_millis(250);
    ui.update();
    buf = display.getBuffer();
    assert(buf.find("ERROR!") != std::string::npos);
    assert(buf.find("OVERTEMP") != std::string::npos);

    std::cout << "UI Rendering and Content test PASSED" << std::endl;
}

void test_cv_termination() {
    std::cout << "Testing CV Termination (10s threshold)..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048); // ~28V
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    charger.setup();

    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // Simulate battery reached target voltage and current dropped
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(28.5 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(0.05 / CURRENT_RAW_TO_A)); // 0.05 < 0.1 threshold

    // Should take 10 seconds of this condition
    for(int i=0; i<90; i++) {
        advance_millis(100);
        charger.update(0.1);
        assert(charger.state() == CHARGING);
    }

    // Wait a bit more to cross 10s
    for(int i=0; i<20; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGED_COMPLETE) break;
    }
    assert(charger.state() == CHARGED_COMPLETE);
    std::cout << "CV Termination test PASSED" << std::endl;
}

void test_pwm_control_logic() {
    std::cout << "Testing PWM Control Logic (Steps)..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3958); // ~29V
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 1638); // ~1.5A
    charger.setup();

    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }

    int initialPwm = charger.pwmDuty();

    // 1. Test Step UP (CC mode, current below limit)
    // _softStartLimit starts at 0.05 and ramps up.
    // Let's wait for soft start to reach 1.0A
    for(int i=0; i<200; i++) {
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(0.1 / CURRENT_RAW_TO_A));
        charger.update(0.1);
    }

    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(0.5 / CURRENT_RAW_TO_A)); // 0.5A < limit
    int lastPwm = charger.pwmDuty();
    charger.update(0.1);
    assert(charger.pwmDuty() > lastPwm);
    std::cout << "Step UP verified: " << lastPwm << " -> " << charger.pwmDuty() << std::endl;

    // 2. Test Step DOWN FAST (Current above limit)
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.5 / CURRENT_RAW_TO_A)); // 2.5A > 1.5A limit
    // Need to wait for filters to catch up
    for(int i=0; i<100; i++) charger.update(0.01);

    lastPwm = charger.pwmDuty();
    charger.update(0.1);
    if (charger.pwmDuty() >= lastPwm) {
        std::cout << "Step DOWN FAST FAILED. lastPwm: " << lastPwm << " currentPwm: " << charger.pwmDuty() << " iChg: " << charger.iChg() << " limit: " << charger.currentLimit() << std::endl;
    }
    assert(charger.pwmDuty() < lastPwm);
    assert((lastPwm - charger.pwmDuty()) >= PWM_STEP_DOWN_FAST);
    std::cout << "Step DOWN FAST verified: " << lastPwm << " -> " << charger.pwmDuty() << std::endl;

    // 3. Test Step DOWN SLOW (Voltage above limit)
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(29.5 / VOLTAGE_SENSE_FACTOR)); // 29.5V > 29.0V
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(1.4 / CURRENT_RAW_TO_A)); // current OK
    // Wait for filter
    for(int i=0; i<100; i++) charger.update(0.01);

    lastPwm = charger.pwmDuty();
    charger.update(0.1);
    if (charger.pwmDuty() >= lastPwm) {
        std::cout << "Step DOWN SLOW FAILED. lastPwm: " << lastPwm << " currentPwm: " << charger.pwmDuty() << " Vbat: " << charger.vBat() << " Target: " << charger.targetVoltage() << std::endl;
    }
    assert(charger.pwmDuty() < lastPwm);
    assert((lastPwm - charger.pwmDuty()) == PWM_STEP_DOWN_SLOW);
    std::cout << "Step DOWN SLOW verified: " << lastPwm << " -> " << charger.pwmDuty() << std::endl;

    std::cout << "PWM Control Logic test PASSED" << std::endl;
}

void test_internal_resistance() {
    std::cout << "Testing Internal Resistance Estimation..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    charger.setup();

    // 1. Enter CHARGING
    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // 2. Simulate load
    // V_loaded = 25V, I_loaded = 2A
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(25.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.0 / CURRENT_RAW_TO_A));
    for(int i=0; i<50; i++) charger.update(0.02);

    // 3. Trigger periodic check
    // Wait for CHARGE_CHECK_INTERVAL_MS (30s)
    advance_millis(31000);
    charger.update(0.1);
    assert(charger.state() == PAUSED_CHECK_VOLTAGE);

    // 4. Provide unloaded voltage
    // V_unloaded = 24.5V.  R = (25.0 - 24.5) / 2.0 = 0.25 ohm
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.5 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);

    // Wait for PAUSE_SETTLE_MS (1s)
    advance_millis(1100);
    charger.update(0.1);

    std::cout << "Estimated Rbat: " << charger.batteryInternalResistance() << " ohm" << std::endl;
    assert(std::abs(charger.batteryInternalResistance() - 0.25) < 0.05);

    std::cout << "Internal Resistance test PASSED" << std::endl;
}

int main() {
    test_ema_filter();
    test_calculate_temp();
    test_psu_cooldown();
    test_integrators();
    test_ui_button();
    test_analog_read_averaged();
    test_storage();
    test_calibration();
    test_fan_control();
    test_setpoints();
    test_reset_logic();
    test_ui_render();
    test_internal_resistance();
    test_cv_termination();
    test_pwm_control_logic();
    return 0;
}
