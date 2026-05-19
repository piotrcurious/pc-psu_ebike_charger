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
extern void advance_millis(unsigned long ms);

void test_voc_estimation() {
    std::cout << "Testing Voc Estimation Accuracy..." << std::endl;
    Charger charger;
    // target 28V, limit 5A
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 4000);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    charger.setup();

    // Simulate some charging with resistance measurement
    // Let's manually set internal resistance for test if we could,
    // but we have to trigger the measurement.

    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }

    // Loaded state
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(26.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.0 / CURRENT_RAW_TO_A));
    for(int i=0; i<20; i++) charger.update(0.05);

    // Trigger pause
    advance_millis(CHARGE_CHECK_INTERVAL_MS + 1000);
    for(int i=0; i<100; i++) {
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(26.0 / VOLTAGE_SENSE_FACTOR));
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(2.0 / CURRENT_RAW_TO_A));
        charger.update(0.1);
    }
    std::cout << "State before pause check: " << (int)charger.state() << std::endl;
    assert(charger.state() == PAUSED_CHECK_VOLTAGE);

    // Unloaded
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(25.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    advance_millis(1100);
    charger.update(0.1);

    float r = charger.batteryInternalResistance();
    std::cout << "Measured R: " << r << " ohm" << std::endl;
    // R = (26 - 25) / 2 = 0.5 ohm
    assert(std::abs(r - 0.5) < 0.1);

    // Back to charging
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(26.5 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(1.0 / CURRENT_RAW_TO_A));
    for(int i=0; i<20; i++) charger.update(0.05);

    float voc = charger.vBatOC();
    std::cout << "V_bat: " << charger.vBat() << " I_chg: " << charger.iChg() << " Est Voc: " << voc << std::endl;
    // Expected Voc = 26.5 - (1.0 * 0.5) = 26.0
    assert(std::abs(voc - 26.0) < 0.2);

    std::cout << "Voc Estimation test PASSED" << std::endl;
}

void test_adaptive_convergence() {
    std::cout << "Testing Adaptive PWM Convergence Speed..." << std::endl;
    Charger charger;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048); // ~28V
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 2048); // ~2.5A
    charger.setup();

    for(int i=0; i<300; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }

    // Wait for soft start
    for(int i=0; i<100; i++) {
        charger.update(0.1);
    }

    int startPwm = charger.pwmDuty();
    int count = 0;
    // Target 2.5A. Current is 0 initially.
    while(charger.iChg() < 2.4 && count < 500) {
        // Simple current model: current = PWM / 50
        float current = (float)charger.pwmDuty() / 50.0f;
        setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(current / CURRENT_RAW_TO_A));
        charger.update(0.02);
        count++;
    }

    std::cout << "Reached 2.4A in " << count << " iterations. Final PWM: " << charger.pwmDuty() << std::endl;
    assert(count < 100); // Should be fast with adaptive stepping
    std::cout << "Adaptive convergence test PASSED" << std::endl;
}

int main() {
    test_voc_estimation();
    test_adaptive_convergence();
    return 0;
}
