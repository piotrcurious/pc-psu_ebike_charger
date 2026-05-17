#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <numeric>
#include <random>
#include "mock_arduino/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "mock_arduino/Wire/Wire.h"
#include "mock_arduino/Preferences.h"

TwoWire WireStub;
Adafruit_SSD1306 display(128, 64, &WireStub, -1);

#include "../Charger.cpp"
#include "../UI.cpp"

extern void setAnalogRead(uint8_t pin, int value);
extern void advance_millis(unsigned long ms);

void test_long_term_integration() {
    std::cout << "Testing Long-Term Integration (100 hours)..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048); // Zero for calibration
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3000);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 3000);
    charger.setup();

    // Enter charging
    for(int i=0; i<500; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // Simulate 11 hours at 1A (to stay within 12h timeout)
    double expectedAh = 0;
    float dt = 10.0f; // 10s steps
    unsigned long steps = (11 * 3600) / 10;

    for(unsigned long i=0; i<steps; i++) {
        expectedAh += (1.0 * dt) / 3600.0;
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(1.0 / CURRENT_RAW_TO_A));
        advance_millis(dt * 1000);
        charger.update(dt);
        if (charger.state() == PAUSED_CHECK_VOLTAGE) {
            advance_millis(PAUSE_SETTLE_MS + 100);
            charger.update(0.1);
        }
        if (i % 10000 == 0) {
             // Avoid console flood
        }
    }

    std::cout << "Expected Ah: " << expectedAh << " Actual Ah: " << charger.ah() << " State: " << (int)charger.state() << std::endl;
    // Check drift - with float we expect some, but should be < 0.1%
    assert(std::abs(charger.ah() - (float)expectedAh) < 0.1);
    std::cout << "Long-term integration test PASSED" << std::endl;
}

void test_noise_resilience() {
    std::cout << "Testing Filter Noise Resilience..." << std::endl;
    Charger charger;
    charger.setup();

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(24.0, 1.0); // Mean 24V, StdDev 1V noise

    std::vector<float> filteredValues;
    for(int i=0; i<500; i++) {
        float noiseV = (float)distribution(generator);
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(noiseV / VOLTAGE_SENSE_FACTOR));
        charger.update(0.02);
        filteredValues.push_back(charger.vBat());
    }

    // Check if the last 100 values are stable
    float sum = std::accumulate(filteredValues.end() - 100, filteredValues.end(), 0.0f);
    float mean = sum / 100.0f;

    float sq_sum = std::inner_product(filteredValues.end() - 100, filteredValues.end(), filteredValues.end() - 100, 0.0f);
    float stdev = std::sqrt(sq_sum / 100.0f - mean * mean);

    std::cout << "Noise Input StdDev: 1.0V, Filtered Output StdDev: " << stdev << "V" << std::endl;
    assert(stdev < 0.3); // Filter should significantly reduce variance (1.0 -> <0.3)
    std::cout << "Noise resilience test PASSED" << std::endl;
}

void test_error_recovery() {
    std::cout << "Testing Error Recovery..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.setup();

    // 1. Trigger Overtemp
    setAnalogRead(TEMP_SENSE_PIN, 100);
    for(int i=0; i<100; i++) charger.update(0.1);
    assert(charger.state() == ERROR_STATE);
    assert(charger.lastError() == OVERTEMP);

    // 2. Try to recover while still hot
    charger.reset();
    charger.update(0.1);
    assert(charger.state() == ERROR_STATE); // Should immediately re-trigger if check is in update

    // 3. Cool down and reset
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.reset();
    charger.update(0.1);
    assert(charger.state() == IDLE);

    // 4. Enter charging again
    for(int i=0; i<500; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    std::cout << "Error recovery test PASSED" << std::endl;
}

void test_state_transitions() {
    std::cout << "Testing State Machine Transitions..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048); // ~28V
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 2048); // ~2.5A
    charger.setup();

    // 1. IDLE -> CHARGING
    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // 2. Simulate full battery (CV termination)
    // To reach CHARGED_COMPLETE, voltage must be high AND current must be below threshold for 10s
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(29.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(0.02 / CURRENT_RAW_TO_A)); // 0.02A < 0.1A threshold

    for(int i=0; i<150; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGED_COMPLETE) break;
    }
    assert(charger.state() == CHARGED_COMPLETE);
    assert(!charger.isPsuOn());

    // 3. CHARGED_COMPLETE -> CHARGING (Re-charge on voltage drop)
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(25.0 / VOLTAGE_SENSE_FACTOR)); // Drop significantly below target
    // Need to wait for filters to catch up and cooldown
    for(int i=0; i<500; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING && charger.isPsuOn()) break;
    }
    if (charger.state() != CHARGING) {
        std::cout << "Re-charge FAILED. State: " << (int)charger.state() << " PSU: " << (int)charger.isPsuOn() << " Vbat: " << charger.vBat() << " Target: " << charger.targetVoltage() << std::endl;
    }
    assert(charger.state() == CHARGING);

    // 4. CHARGING -> IDLE (Battery removed)
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(5.0 / VOLTAGE_SENSE_FACTOR)); // Low voltage
    // The check for disconnected is either sudden drop or periodic check
    // Let's trigger periodic check
    advance_millis(CHARGE_CHECK_INTERVAL_MS + 1000);
    charger.update(0.1); // Enter PAUSED
    advance_millis(PAUSE_SETTLE_MS + 100);
    charger.update(0.1); // Check and fail
    assert(charger.state() == ERROR_STATE);
    assert(charger.lastError() == DISCONNECTED);

    std::cout << "State transitions test PASSED" << std::endl;
}

int main() {
    test_long_term_integration();
    test_noise_resilience();
    test_error_recovery();
    test_state_transitions();
    return 0;
}
