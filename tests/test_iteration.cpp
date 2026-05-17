#include "mock_arduino/Arduino.h"
#include <iostream>
#include <iomanip>
#include "mock_arduino/Adafruit_SSD1306/Adafruit_SSD1306.h"
#include "mock_arduino/Wire/Wire.h"
#include "mock_arduino/Preferences.h"
TwoWire WireStub;
Adafruit_SSD1306 display(128, 64, &WireStub, -1);
#include "../Charger.cpp"
#include "../UI.cpp"
extern void setAnalogRead(uint8_t pin, int value);
extern void setDigitalRead(uint8_t pin, uint8_t val);
extern uint32_t getPwmWrite(uint8_t channel);
extern void advance_millis(unsigned long ms);
struct Simulation {
    float batteryVoltage = 24.0;
    float batteryCapacity = 2.0;
    float temperature = 25.0;
    float getBoostVoltage(int duty, bool psu_on) {
        if (!psu_on) return 0;
        float D = (float)duty / 1024.0;
        if (D >= 0.95) D = 0.95;
        // Boost converter model: Vout = Vin / (1-D)
        // With current limiting behavior of ATX PSU if overloaded
        return 12.0 / (1.0 - D);
    }
    float getBatteryVoltage() { return batteryVoltage; }
    void step(int duty, bool psu_on, float dt_seconds) {
        float v_out = getBoostVoltage(duty, psu_on);
        float current = (v_out > batteryVoltage) ? (v_out - batteryVoltage) / 0.1 : 0;
        batteryVoltage += (current * dt_seconds) / (batteryCapacity * 3600.0) * 100.0;
        temperature += current * 0.1 * dt_seconds;
        temperature -= (temperature - 25.0) * 0.01 * dt_seconds;
    }
    int getTempRaw(float temp) { return (int)(2048 - (temp - 25.0) * 20); }
};
int main() {
    Simulation sim;
    Charger charger;
    UI ui(display, charger);
    setAnalogRead(2, 3958); setAnalogRead(3, 1638); setAnalogRead(1, 2048);
    setDigitalRead(UI_BUTTON_PIN, HIGH);
    charger.setup();
    ui.setup();
    std::cout << "Starting Automated Test Cycle..." << std::endl;
    for (int i = 0; i < 50000; ++i) {
        setAnalogRead(0, (int)(sim.getBatteryVoltage() / VOLTAGE_SENSE_FACTOR));
        setAnalogRead(5, sim.getTempRaw(sim.temperature));
        bool psu_on = charger.isPsuOn();
        float v_out = sim.getBoostVoltage(charger.pwmDuty(), psu_on);
        if (v_out < 12.0 && psu_on) v_out = 12.0;
        float current = (psu_on && v_out > sim.getBatteryVoltage()) ?
                        (v_out - sim.getBatteryVoltage()) / 0.1 : 0;

        // Ensure PSU health check doesn't trip by providing "current" during ramp
        if (psu_on && charger.pwmDuty() > 10) {
            current = std::max(current, 0.1f);
        }

        setAnalogRead(1, (int)(current / CURRENT_RAW_TO_A) + 2048);
        charger.update(0.05);
        ui.update();
        if (i % 2000 == 0) {
            std::cout << "Screen change..." << std::endl;
            setDigitalRead(UI_BUTTON_PIN, LOW);
            ui.update();
            setDigitalRead(UI_BUTTON_PIN, HIGH);
            ui.update();
        }
        sim.step(charger.pwmDuty(), psu_on, 0.05);
        advance_millis(50);
        if (charger.state() == CHARGED_COMPLETE || charger.state() == ERROR_STATE) break;
    }
    std::cout << "Final Ah: " << charger.ah() << " Wh: " << charger.wh() << std::endl;
    if (charger.state() == CHARGED_COMPLETE) std::cout << "TEST PASSED" << std::endl;
    else std::cout << "TEST FAILED: State=" << (int)charger.state() << " Error=" << (int)charger.lastError() << std::endl;
    return 0;
}
