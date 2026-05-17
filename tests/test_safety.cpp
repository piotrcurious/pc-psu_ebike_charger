#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>
#include "mock_arduino/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "mock_arduino/Wire/Wire.h"
#include "mock_arduino/Preferences.h"

TwoWire WireStub;
Adafruit_SSD1306 display(128, 64, &WireStub, -1);

#include "../Charger.cpp"
#include "../UI.cpp"

extern void setAnalogRead(uint8_t pin, int value);
extern void setDigitalRead(uint8_t pin, uint8_t val);
extern void advance_millis(unsigned long ms);

void test_overtemp() {
    std::cout << "Testing Over-temperature protection..." << std::endl;
    Charger charger;

    // Initialize mocks
    setAnalogRead(TEMP_SENSE_PIN, 2048); // ~25C
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);

    charger.setup();

    // Normal temp
    charger.update(0.1);
    assert(charger.state() != ERROR_STATE);

    // Over temp
    setAnalogRead(TEMP_SENSE_PIN, 500); // Low raw value = high temperature
    for(int i=0; i<20; i++) {
        charger.update(0.1);
        if (charger.state() == ERROR_STATE) break;
    }

    if (charger.state() == ERROR_STATE && charger.lastError() == OVERTEMP) {
        std::cout << "Over-temperature test PASSED" << std::endl;
    } else {
        std::cout << "Over-temperature test FAILED! State: " << (int)charger.state() << " Error: " << (int)charger.lastError() << std::endl;
    }
}

void test_overcurrent() {
    std::cout << "Testing Over-current protection..." << std::endl;
    Charger charger;

    // Initialize mocks
    setAnalogRead(TEMP_SENSE_PIN, 2048); // ~25C
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3958); // ~29V
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 1638); // ~1.5A limit

    charger.setup();

    charger.update(0.1);

    // Transition from IDLE to CHARGING
    for(int i=0; i<200; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }

    if (charger.state() != CHARGING) {
        std::cout << "State is " << (int)charger.state() << " PSU is " << (int)charger.isPsuOn() << " Vbat=" << charger.vBat() << std::endl;
    }
    assert(charger.state() == CHARGING);

    // Simulate huge current
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(10.0 / CURRENT_RAW_TO_A));
    for(int i=0; i<10; i++) {
        charger.update(0.1);
        if (charger.state() == ERROR_STATE) break;
    }

    if (charger.state() == ERROR_STATE && charger.lastError() == OVERCURRENT) {
        std::cout << "Over-current test PASSED" << std::endl;
    } else {
        std::cout << "Over-current test FAILED! State: " << (int)charger.state() << " Error: " << (int)charger.lastError() << std::endl;
    }
}

void test_capacity_limit() {
    std::cout << "Testing Capacity Limit protection..." << std::endl;
    Charger charger;

    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3958);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 1638);

    charger.setup();

    for(int i=0; i<200; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.0 / CURRENT_RAW_TO_A));

    // Inject high Ah
    // Since we can't easily inject into private member, we simulate time or many updates
    // MAX_CHARGE_AH_LIMIT is 20.0
    // 2A for 12 hours = 24Ah
    for(int i=0; i<12; i++) {
        advance_millis(3600000);
        charger.update(3600.0);
        if (charger.state() == PAUSED_CHECK_VOLTAGE) {
            advance_millis(PAUSE_SETTLE_MS + 100);
            charger.update(0.1);
        }
        if (charger.state() == ERROR_STATE) break;
    }

    if (charger.state() == ERROR_STATE && charger.lastError() == CAPACITY_LIMIT) {
        std::cout << "Capacity limit test PASSED" << std::endl;
    } else {
        std::cout << "Capacity limit test FAILED! State: " << (int)charger.state() << " Ah: " << charger.ah() << std::endl;
    }
}

void test_timeout() {
    std::cout << "Testing Timeout protection..." << std::endl;
    Charger charger;

    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3958);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 1638);

    charger.setup();

    for(int i=0; i<200; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // MAX_CHARGE_TIME_MS is 12 hours
    advance_millis(13 * 3600 * 1000UL);
    charger.update(0.1);

    if (charger.state() == ERROR_STATE && charger.lastError() == TIMEOUT) {
        std::cout << "Timeout test PASSED" << std::endl;
    } else {
        std::cout << "Timeout test FAILED! State: " << (int)charger.state() << std::endl;
    }
}

void test_sensor_fault() {
    std::cout << "Testing Sensor Fault protection..." << std::endl;
    Charger charger;
    charger.setup();

    // ADC Overvoltage on battery sense pin
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, 4095); // 3.3V > 3.25V threshold
    for(int i=0; i<32; i++) charger.update(0.1);

    if (charger.state() == ERROR_STATE && charger.lastError() == SENSOR_FAULT) {
        std::cout << "Sensor fault test PASSED" << std::endl;
    } else {
        std::cout << "Sensor fault test FAILED! State: " << (int)charger.state() << " Error: " << (int)charger.lastError() << std::endl;
    }
}

int main() {
    test_overtemp();
    test_overcurrent();
    test_capacity_limit();
    test_timeout();
    test_sensor_fault();
    return 0;
}
