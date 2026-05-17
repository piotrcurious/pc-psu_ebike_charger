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

void test_sudden_disconnect() {
    std::cout << "Testing Sudden Disconnect protection..." << std::endl;
    Charger charger;

    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3958);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 1638);

    charger.setup();

    // Get into charging state
    for(int i=0; i<200; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // Simulate steady charging - need to ramp up PWM
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048 + (int)(0.6 / CURRENT_RAW_TO_A));
    for(int i=0; i<200; i++) {
        charger.update(0.1);
    }

    // Ensure PWM is significant
    std::cout << "PWM Duty: " << charger.pwmDuty() << " iChg: " << charger.iChg() << std::endl;

    // Sudden drop to zero current
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    // Use very small dt to avoid filter lag in test
    for(int i=0; i<1000; i++) {
        charger.update(0.001);
        if (charger.state() == ERROR_STATE) break;
    }
    std::cout << "After drop - iChg: " << charger.iChg() << " State: " << charger.state() << std::endl;

    if (charger.state() == ERROR_STATE && charger.lastError() == DISCONNECTED) {
        std::cout << "Sudden disconnect test PASSED" << std::endl;
    } else {
        std::cout << "Sudden disconnect test FAILED! State: " << (int)charger.state() << " Error: " << (int)charger.lastError() << std::endl;
    }
}

void test_psu_failure() {
    std::cout << "Testing PSU Failure detection..." << std::endl;
    Charger charger;

    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 3958);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 1638);

    charger.setup();

    // Start charging
    for(int i=0; i<200; i++) {
        advance_millis(100);
        charger.update(0.1);
        if (charger.state() == CHARGING) break;
    }
    assert(charger.state() == CHARGING);

    // Maintain zero current despite being in CHARGING state
    // We need to wait 5 seconds (from _chargeStartTime)
    advance_millis(6000);
    // And ensure PWM is high enough
    // The health check needs _currentPwmDuty > (MAX_PWM_DUTY / 2)
    // We can simulate this by telling charger current is low for many cycles
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    for(int i=0; i<300; i++) {
        charger.update(0.1);
        if (charger.state() == ERROR_STATE) break;
    }

    if (charger.state() == ERROR_STATE && charger.lastError() == DISCONNECTED) {
        std::cout << "PSU failure test PASSED" << std::endl;
    } else {
        std::cout << "PSU failure test FAILED! State: " << (int)charger.state() << " PWM: " << charger.pwmDuty() << std::endl;
    }
}

int main() {
    test_sudden_disconnect();
    test_psu_failure();
    return 0;
}
