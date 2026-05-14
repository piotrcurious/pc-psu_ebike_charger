#include "mock_arduino/Arduino.h"
#include <iostream>
#include <iomanip>
#include "../esp32-c3_oled.ino"

extern void setAnalogRead(uint8_t pin, int value);
extern void setDigitalRead(uint8_t pin, uint8_t val);
extern uint32_t getPwmWrite(uint8_t channel);
extern void advance_millis(unsigned long ms);

int main() {
    setAnalogRead(2, 3958); setAnalogRead(3, 1638); setAnalogRead(1, 2048);
    setAnalogRead(0, (int)(24.0 / VOLTAGE_SENSE_FACTOR));
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    setup();

    std::cout << "Starting UI Test..." << std::endl;
    std::cout << "Initial screen: " << currentScreen << std::endl;

    for (int i = 1; i <= 3; ++i) {
        std::cout << "Pressing button..." << std::endl;
        setDigitalRead(UI_BUTTON_PIN, LOW);
        loop(); // Detect press
        setDigitalRead(UI_BUTTON_PIN, HIGH);
        loop(); // Detect release
        std::cout << "Current screen: " << currentScreen << std::endl;
    }

    if (currentScreen == 3) std::cout << "PASSED: Cycled through screens" << std::endl;
    else std::cout << "FAILED: Current screen is " << currentScreen << std::endl;

    return 0;
}
