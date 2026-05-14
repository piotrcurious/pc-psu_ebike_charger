#include "mock_arduino/Arduino.h"
#include <iostream>
#include <random>
#include "../esp32-c3_oled.ino"
extern void setAnalogRead(uint8_t pin, int value);
extern uint32_t getPwmWrite(uint8_t channel);
extern void advance_millis(unsigned long ms);
struct Simulation {
    float batteryVoltage = 24.0;
    float batteryCapacity = 0.5;
    std::default_random_engine generator;
    std::normal_distribution<double> noise;
    Simulation() : noise(0.0, 10.0) {}
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
    int getNoisyRaw(float physical, float factor) {
        int raw = (int)(physical / factor);
        return raw + (int)noise(generator);
    }
};
int main() {
    Simulation sim;
    setAnalogRead(2, 3958); setAnalogRead(3, 1638); setAnalogRead(1, 2048);
    setup();
    for (int i = 0; i < 5000; ++i) {
        setAnalogRead(0, sim.getNoisyRaw(sim.batteryVoltage, VOLTAGE_SENSE_FACTOR));
        bool psu_on = (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON);
        float current = (psu_on && sim.getBoostVoltage(getPwmWrite(0), psu_on) > sim.batteryVoltage) ?
                        (sim.getBoostVoltage(getPwmWrite(0), psu_on) - sim.batteryVoltage) / 0.6 : 0;
        setAnalogRead(1, sim.getNoisyRaw(current, CURRENT_RAW_TO_A) + 2048);
        loop();
        sim.step(currentPwmDutyCycle, psu_on, 0.07);
        advance_millis(70);
    }
    std::cout << "Noisy Simulation Complete." << std::endl;
    return 0;
}
