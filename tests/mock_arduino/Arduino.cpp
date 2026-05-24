#include "Arduino.h"
#include "esp_task_wdt.h"
#include <iostream>
SerialMock Serial;
static unsigned long _millis = 0;
unsigned long millis() { return _millis; }
void delay(unsigned long ms) { _millis += ms; }
void delayMicroseconds(unsigned int us) {}
void advance_millis(unsigned long ms) { _millis += ms; }
static int _analogValues[64] = {0};
static int _analogQueue[64][128];
static int _analogQueueHead[64] = {0};
static int _analogQueueSize[64] = {0};

int analogRead(uint8_t pin) {
    if (pin < 64 && _analogQueueSize[pin] > 0) {
        int val = _analogQueue[pin][_analogQueueHead[pin]];
        _analogQueueHead[pin] = (_analogQueueHead[pin] + 1) % 128;
        _analogQueueSize[pin]--;
        // std::cout << "analogRead(queue) pin=" << (int)pin << " val=" << val << std::endl;
        return val;
    }
    // std::cout << "analogRead(fixed) pin=" << (int)pin << " val=" << _analogValues[pin] << std::endl;
    return (pin < 64) ? _analogValues[pin] : 0;
}
void setAnalogRead(uint8_t pin, int value) { if (pin < 64) _analogValues[pin] = value; }
void queueAnalogRead(uint8_t pin, int value) {
    if (pin < 64 && _analogQueueSize[pin] < 128) {
        int tail = (_analogQueueHead[pin] + _analogQueueSize[pin]) % 128;
        _analogQueue[pin][tail] = value;
        _analogQueueSize[pin]++;
        // // std::cout << "queueAnalogRead pin=" << (int)pin << " val=" << value << " size=" << _analogQueueSize[pin] << std::endl;
    }
}

void clearAnalogQueue(uint8_t pin) {
    if (pin < 64) {
        _analogQueueSize[pin] = 0;
        _analogQueueHead[pin] = 0;
    }
}
static uint8_t _digitalValues[64] = {0};
void pinMode(uint8_t pin, uint8_t mode) {}
void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) _digitalValues[pin] = val; }
int digitalRead(uint8_t pin) { return (pin < 64) ? _digitalValues[pin] : 0; }
void setDigitalRead(uint8_t pin, uint8_t val) { if (pin < 64) _digitalValues[pin] = val; }
void analogReadResolution(int res) {}
void analogSetPinAttenuation(int pin, int atten) {}
uint32_t analogReadMilliVolts(uint8_t pin) {
    int raw = analogRead(pin);
    uint32_t val = (uint32_t)((float)raw * 3300.0f / 4095.0f);
    // std::cout << "analogReadMilliVolts pin=" << (int)pin << " raw=" << raw << " mv=" << val << std::endl;
    return val;
}
static uint32_t _pwmValues[16] = {0};
void ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits) {}
void ledcAttachPin(uint8_t pin, uint8_t channel) {}
void ledcWrite(uint8_t channel, uint32_t duty) { if (channel < 16) _pwmValues[channel] = duty; }
uint32_t getPwmWrite(uint8_t channel) { return (channel < 16) ? _pwmValues[channel] : 0; }
#include "Wire/Wire.h"
TwoWire Wire;

#include "Preferences.h"
std::map<std::string, float> Preferences::_data;
void esp_task_wdt_init(int timeout, bool panic) {}
void esp_task_wdt_add(void* handle) {}
void esp_task_wdt_reset() {}
