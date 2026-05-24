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
extern void clearAnalogQueue(uint8_t pin);
extern void setDigitalRead(uint8_t pin, uint8_t val);
extern void advance_millis(unsigned long ms);

void test_ema_filter() {
    std::cout << "Testing EMAFilter..." << std::endl;
    EMAFilter<float> filter(0.5);
    assert(filter.value() == 0);

    filter.update(10.0);
    assert(filter.value() == 10.0);

    filter.update(20.0);
    assert(filter.value() == 15.0);

    filter.reset();
    std::cout << "EMAFilter test PASSED" << std::endl;
}

void test_calculate_temp() {
    std::cout << "Testing Temperature Calculation..." << std::endl;
    Charger charger;
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.update(0.1);
    float t = charger.temp();
    std::cout << "Temp at 2048: " << t << " C" << std::endl;
    assert(t > 24.0 && t < 26.0);

    setAnalogRead(TEMP_SENSE_PIN, 500);
    for(int i=0; i<100; i++) charger.update(0.1);
    t = charger.temp();
    std::cout << "Temp at 500: " << t << " C" << std::endl;
    assert(t > 50.0);

    setAnalogRead(TEMP_SENSE_PIN, 3800);
    for(int i=0; i<100; i++) charger.update(0.1);
    t = charger.temp();
    std::cout << "Temp at 3800: " << t << " C" << std::endl;
    assert(t < 10.0);

    std::cout << "Temperature Calculation test PASSED" << std::endl;
}

void test_temp_safety() {
    std::cout << "Testing Temperature Safety (High and Low)..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(TEMP_SENSE_PIN, 2048); // 25C
    charger.setup();

    // High Temp
    setAnalogRead(TEMP_SENSE_PIN, 400); // Very hot
    for(int i=0; i<100; i++) charger.update(0.1);
    assert(charger.state() == ERROR_STATE);

    charger.reset();
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.update(0.1);
    assert(charger.state() == IDLE);

    // Low Temp
    setAnalogRead(TEMP_SENSE_PIN, 4000); // Very cold (~ -10C)
    for(int i=0; i<100; i++) charger.update(0.1);
    assert(charger.state() == ERROR_STATE);

    std::cout << "Temperature Safety test PASSED" << std::endl;
}

void test_calibration_persistence() {
    std::cout << "Testing Calibration Persistence..." << std::endl;
    Preferences::clear();
    // Calibrate once
    {
        Charger charger;
        clearAnalogQueue(CURRENT_SENSE_AMP_PIN);
        // We provide 100 samples of 1800mV
        for (int i = 0; i < 100; i++) queueAnalogRead(CURRENT_SENSE_AMP_PIN, (int)(1800.0 * 4095.0 / 3300.0));
        charger.setup();
        std::cout << "Charger offset: " << charger.currentOffsetRaw() << std::endl;
        assert(std::abs(charger.currentOffsetRaw() - 1800) < 10);
    }

    // New charger instance should load it
    {
        Charger charger2;
        clearAnalogQueue(CURRENT_SENSE_AMP_PIN);
        charger2.setup();
        std::cout << "Charger2 offset: " << charger2.currentOffsetRaw() << std::endl;
        assert(std::abs(charger2.currentOffsetRaw() - 1800) < 10);

        // Recalibrate
        clearAnalogQueue(CURRENT_SENSE_AMP_PIN);
        for (int i = 0; i < 100; i++) queueAnalogRead(CURRENT_SENSE_AMP_PIN, (int)(1900.0 * 4095.0 / 3300.0));
        charger2.calibrateCurrentSensor();
        std::cout << "Charger2 recalibrated offset: " << charger2.currentOffsetRaw() << std::endl;
        assert(std::abs(charger2.currentOffsetRaw() - 1900) < 10);
    }

    {
        Charger charger3;
        clearAnalogQueue(CURRENT_SENSE_AMP_PIN);
        charger3.setup();
        std::cout << "Charger3 offset: " << charger3.currentOffsetRaw() << std::endl;
        assert(std::abs(charger3.currentOffsetRaw() - 1900) < 10);
    }

    std::cout << "Calibration Persistence test PASSED" << std::endl;
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

    // Clear error
    charger.reset();
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));

    // Try to restart immediately
    charger.update(0.1);
    assert(!charger.isPsuOn());

    // Wait cooldown
    advance_millis(PSU_RESTART_COOLDOWN_MS + 1000);
    charger.update(0.1);
    assert(charger.isPsuOn());

    std::cout << "PSU Cooldown test PASSED" << std::endl;
}

void test_integrators() {
    std::cout << "Testing Integrators..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3000);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 3000);
    charger.setup();

    int mv_2A = (int)(charger.currentOffsetRaw() + 2.0 * ACS712_SENSITIVITY * 1000.0);
    int raw_2A = (int)((float)mv_2A * 4095.0f / 3300.0f);

    for(int i=0; i<400; i++) {
        setAnalogRead(CURRENT_SENSE_AMP_PIN, raw_2A);
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    for(int i=0; i<3600; i++) {
        setAnalogRead(CURRENT_SENSE_AMP_PIN, raw_2A);
        advance_millis(1000);
        charger.update(1.0);
        if (charger.state() == PAUSED_CHECK_VOLTAGE) {
            advance_millis(PAUSE_SETTLE_MS + 100);
            setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
            charger.update(0.1);
        }
        if (charger.state() == ERROR_STATE) {
            std::cout << "Integrators test FAILED: entered ERROR_STATE. Last error: " << (int)charger.lastError() << " iChg: " << charger.iChg() << " limit: " << charger.currentLimit() << std::endl;
            assert(false);
        }
    }

    std::cout << "Ah after 1 hour at 2A: " << charger.ah() << std::endl;
    assert(std::abs(charger.ah() - 2.0) < 0.1);

    std::cout << "Wh after 1 hour at 2A/24V: " << charger.wh() << std::endl;
    assert(std::abs(charger.wh() - 48.0) < 2.0);

    std::cout << "Integrators test PASSED" << std::endl;
}

void test_ui_button() {
    std::cout << "Testing UI Button Debounce and Long-press..." << std::endl;
    Charger charger;
    UI ui(display, charger);
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    ui.setup();

    setDigitalRead(UI_BUTTON_PIN, LOW);
    advance_millis(100);
    ui.update();
    advance_millis(100);
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    advance_millis(100);
    ui.update();

    std::cout << "UI Button test PASSED" << std::endl;
}

void test_analog_read_averaged() {
    std::cout << "Testing analogReadAveraged..." << std::endl;
    Charger charger;
    clearAnalogQueue(BAT_VOLTAGE_SENSE_PIN);
    for (int i = 0; i < 16; i++) {
        queueAnalogRead(BAT_VOLTAGE_SENSE_PIN, 1000);
    }
    clearAnalogQueue(CURRENT_SENSE_AMP_PIN);
    for (int i = 0; i < 100; i++) {
        queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    }
    for (int i = 0; i < 16; i++) {
        queueAnalogRead(BAT_VOLTAGE_SENSE_PIN, 1000 + i);
    }
    for (int i = 0; i < 16; i++) queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(TEMP_SENSE_PIN, 2048);

    charger.setup();

    for (int i = 0; i < 16; i++) {
        queueAnalogRead(BAT_VOLTAGE_SENSE_PIN, 1000 + i);
    }
    for (int i = 0; i < 16; i++) queueAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    for (int i = 0; i < 16; i++) queueAnalogRead(TEMP_SENSE_PIN, 2048);

    charger.update(0.1);

    float expectedV = (1000.0f * 3300.0f / 4095.0f) * 0.001f * VOLTAGE_DIVIDER_RATIO;
    assert(std::abs(charger.vBat() - expectedV) < 0.5);
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
    assert(std::abs(storage2.lifetimeAh() - 1.23f) < 0.001);
    assert(std::abs(storage2.lifetimeWh() - 45.6f) < 0.001);
    std::cout << "ChargerStorage test PASSED" << std::endl;
}

void test_fan_control() {
    std::cout << "Testing Fan Control..." << std::endl;
    Charger charger;
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.update(0.1);
    assert(charger.fanDuty() == 0);

    setAnalogRead(TEMP_SENSE_PIN, 500);
    for(int i=0; i<100; i++) charger.update(0.1);
    assert(charger.fanDuty() == MAX_FAN_DUTY);

    std::cout << "Fan Control test PASSED" << std::endl;
}

void test_cv_termination() {
    std::cout << "Testing CV Termination (10s threshold)..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3000);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 4000);
    charger.setup();

    int mv_offset = charger.currentOffsetRaw();
    int mv_005A = (int)(mv_offset + 0.05 * ACS712_SENSITIVITY * 1000.0);
    int raw_005A = (int)((float)mv_005A * 4095.0f / 3300.0f);

    for(int i=0; i<500; i++) {
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
        setAnalogRead(CURRENT_SENSE_AMP_PIN, raw_005A + 500); // More current to enter charging
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(charger.targetVoltage() / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, raw_005A);

    for(int i=0; i<150; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGED_COMPLETE) break;
    }
    if (charger.state() != CHARGED_COMPLETE) {
        std::cout << "CV Termination FAILED. State: " << (int)charger.state() << " iChg: " << charger.iChg() << " Vbat: " << charger.vBat() << " Target: " << charger.targetVoltage() << std::endl;
    }
    assert(charger.state() == CHARGED_COMPLETE);
    std::cout << "CV Termination test PASSED" << std::endl;
}

int main() {
    test_ema_filter();
    test_calculate_temp();
    test_temp_safety();
    test_calibration_persistence();
    test_psu_cooldown();
    test_integrators();
    test_ui_button();
    test_analog_read_averaged();
    test_storage();
    test_fan_control();
    test_cv_termination();
    return 0;
}
