#include "mock_arduino/Arduino.h"
#include <iostream>
#include <iomanip>
#include "../esp32-c3_oled.ino"
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
        return 12.0 / (1.0 - D);
    }
    void step(int duty, bool psu_on, float dt_seconds) {
        float v_out = getBoostVoltage(duty, psu_on);
        float current = (v_out > batteryVoltage) ? (v_out - batteryVoltage) / 0.6 : 0;
        batteryVoltage += (current * dt_seconds) / (batteryCapacity * 3600.0) * 100.0;
    }
};
int main() {
    Simulation sim;
    setAnalogRead(2, 3958); setAnalogRead(3, 1638); setAnalogRead(1, 2048);
    setup();
    for (int i = 0; i < 5000; ++i) {
        int rawBatV = (int)(sim.batteryVoltage / VOLTAGE_SENSE_FACTOR);
        setAnalogRead(0, rawBatV);
        bool psu_on = (digitalRead(ATX_PS_ON_PIN) == HIGH);
        float v_out = sim.getBoostVoltage(getPwmWrite(0), psu_on);
        float current = (v_out > sim.batteryVoltage) ? (v_out - sim.batteryVoltage) / 0.6 : 0;
        int rawI = (int)(current / CURRENT_RAW_TO_A) + currentSensorOffsetRaw;
        setAnalogRead(1, rawI);
        loop();
        sim.step(currentPwmDutyCycle, psu_on, 0.07);
        advance_millis(70);
    }
    std::cout << "CV Simulation Complete." << std::endl;
    return 0;
}
