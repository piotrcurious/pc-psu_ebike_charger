#include "mock_arduino/Arduino.h"
#include <iostream>
#include <cassert>
#include "mock_arduino/LovyanGFX.hpp"
#include "mock_arduino/Wire/Wire.h"
#include "mock_arduino/Preferences.h"

#include "../Charger.cpp"
#include "../UI_TFT.cpp"

extern void setAnalogRead(uint8_t pin, int value);
extern void advance_millis(unsigned long ms);

void test_tft_ui_render() {
    std::cout << "Testing TFT UI Rendering (No crash)..." << std::endl;
    setAnalogRead(BAT_VOLTAGE_SENSE_PIN, 2048);
    setAnalogRead(CURRENT_SENSE_AMP_PIN, 2048);
    setAnalogRead(TEMP_SENSE_PIN, 2048);
    setAnalogRead(DESIRED_VOLTAGE_SET_PIN, 2048);
    setAnalogRead(DESIRED_CURRENT_SET_PIN, 2048);
    LGFX tft;
    Charger charger;
    UI_TFT ui(tft, charger);


    charger.setup();
    ui.setup();

    // Cycle all screens
    for(int i=0; i<6; i++) {
        std::cout << "Rendering TFT Screen " << i << std::endl;
        advance_millis(200);
        ui.update();

        // Sim button press to change screen
        setDigitalRead(UI_BUTTON_PIN, LOW);
        advance_millis(100);
        ui.update();
        setDigitalRead(UI_BUTTON_PIN, HIGH);
        advance_millis(100);
        ui.update();
    }

    // Test Error state
    for(int i=0; i<300; i++) {
        setAnalogRead(BAT_VOLTAGE_SENSE_PIN, 4095); // SF
        charger.update(0.1);
        if (charger.state() == ERROR_STATE) break;
    }
    if (charger.state() != ERROR_STATE) {
        std::cout << "State: " << (int)charger.state() << " Temp: " << charger.temp() << std::endl;
    }
    assert(charger.state() == ERROR_STATE);
    ui.update();

    std::cout << "TFT UI Render test PASSED" << std::endl;
}

int main() {
    test_tft_ui_render();
    return 0;
}
