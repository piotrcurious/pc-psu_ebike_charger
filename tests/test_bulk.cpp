#include "mock_arduino/Arduino.h"
#include <iostream>
#include <iomanip>

// Include the charger code
#include "../esp32-c3_oled.ino"

// External access to mock functions
extern void setAnalogRead(uint8_t pin, int value);
extern uint32_t getPwmWrite(uint8_t channel);
extern void advance_millis(unsigned long ms);

struct Simulation {
    float batteryVoltage = 24.0;
    float batteryCapacity = 1.0;
    float batteryInternalResistance = 0.1;
    float vin = 12.0;

    float getBoostVoltage(int duty) {
        float D = (float)duty / 1024.0;
        if (D >= 0.95) D = 0.95;
        return vin / (1.0 - D);
    }

    void step(int duty, float dt_seconds) {
        float v_out = getBoostVoltage(duty);
        float current = 0;
        if (v_out > batteryVoltage) {
            current = (v_out - batteryVoltage) / (batteryInternalResistance + 0.5);
        }
        batteryVoltage += (current * dt_seconds) / (batteryCapacity * 3600.0) * 100.0;
    }
};

int main() {
    Simulation sim;
    setAnalogRead(2, 3958); // set V to 29V
    setAnalogRead(3, 1638); // set I to 2A
    setAnalogRead(1, 2048);
    setup();

    std::cout << "Starting Bulk Simulation..." << std::endl;
    std::cout << "Time(s), State, BatV(V), ChgI(A), PWM" << std::endl;

    for (int i = 0; i < 10000; ++i) {
        int rawBatV = (int)(sim.batteryVoltage / VOLTAGE_SENSE_FACTOR);
        setAnalogRead(0, rawBatV);

        float v_out = sim.getBoostVoltage(getPwmWrite(0));
        float current = 0;
        if (v_out > sim.batteryVoltage) {
            current = (v_out - sim.batteryVoltage) / (0.1 + 0.5);
        }
        int rawI = (int)(current / CURRENT_RAW_TO_A) + currentSensorOffsetRaw;
        setAnalogRead(1, rawI);

        loop();

        if (i % 500 == 0) {
            std::cout << std::fixed << std::setprecision(2)
                      << (float)millis() / 1000.0 << ", "
                      << (int)chargerState << ", "
                      << batteryVoltageFiltered << ", "
                      << chargingCurrentFiltered << ", "
                      << currentPwmDutyCycle << std::endl;
        }

        sim.step(currentPwmDutyCycle, 0.05 + 0.02);
        advance_millis(70);
    }

    return 0;
}
