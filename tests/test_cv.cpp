#include "mock_arduino/Arduino.h"
#include <iostream>
#include <iomanip>
#include "mock_arduino/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "mock_arduino/Wire/Wire.h"
TwoWire WireStub;
Adafruit_SSD1306 display(128, 64, &WireStub, -1);
#include "../Charger.cpp"
#include "../UI.cpp"
extern void setAnalogRead(uint8_t pin, int value);
extern uint32_t getPwmWrite(uint8_t channel);
extern void advance_millis(unsigned long ms);
struct Simulation {
    float batteryVoltage = 28.5;
    float batteryCapacity = 0.01;
    float getBoostVoltage(int duty, bool psu_on) {
        if (!psu_on) return 0;
        float vin = 12.0;
        float D = (float)duty / 1024.0;
        if (D >= 0.95) D = 0.95;
        return vin / (1.0 - D);
    }
    void step(int duty, bool psu_on, float dt_seconds) {
        float v_out = getBoostVoltage(duty, psu_on);
        float current = (v_out > batteryVoltage) ? (v_out - batteryVoltage) / 0.6 : 0;
        batteryVoltage += (current * dt_seconds) / (batteryCapacity * 3600.0) * 100.0;
    }
};
int main() {
    Simulation sim;
    Charger charger;
    UI ui(display, charger);
    setAnalogRead(2, 3958); setAnalogRead(3, 1638); setAnalogRead(1, 2048);
    setAnalogRead(5, 2048);
    charger.setup();
    for (int i = 0; i < 5000; ++i) {
        int rawBatV = (int)(sim.batteryVoltage / VOLTAGE_SENSE_FACTOR);
        setAnalogRead(0, rawBatV);
        bool psu_on = charger.isPsuOn();
        float current = (sim.getBoostVoltage(charger.pwmDuty(), psu_on) > sim.batteryVoltage) ? (sim.getBoostVoltage(charger.pwmDuty(), psu_on) - sim.batteryVoltage) / 0.6 : 0;
        setAnalogRead(1, (int)(current / CURRENT_RAW_TO_A) + 2048);
        charger.update(0.07);
        ui.update();
        sim.step(charger.pwmDuty(), psu_on, 0.07);
        advance_millis(70);
    }
    std::cout << "CV Simulation Complete." << std::endl;
    return 0;
}
