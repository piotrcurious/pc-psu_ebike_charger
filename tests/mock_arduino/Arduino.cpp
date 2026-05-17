#include "Arduino.h"
#include "esp_task_wdt.h"
SerialMock Serial;
static unsigned long _millis = 0;
unsigned long millis() { return _millis; }
void delay(unsigned long ms) { _millis += ms; }
void delayMicroseconds(unsigned int us) {}
void advance_millis(unsigned long ms) { _millis += ms; }
static int _analogValues[16] = {0};
static int _analogQueue[16][128];
static int _analogQueueHead[16] = {0};
static int _analogQueueSize[16] = {0};

int analogRead(uint8_t pin) {
    if (pin < 16 && _analogQueueSize[pin] > 0) {
        int val = _analogQueue[pin][_analogQueueHead[pin]];
        _analogQueueHead[pin] = (_analogQueueHead[pin] + 1) % 128;
        _analogQueueSize[pin]--;
        return val;
    }
    return (pin < 16) ? _analogValues[pin] : 0;
}
void setAnalogRead(uint8_t pin, int value) { if (pin < 16) _analogValues[pin] = value; }
void queueAnalogRead(uint8_t pin, int value) {
    if (pin < 16 && _analogQueueSize[pin] < 128) {
        int tail = (_analogQueueHead[pin] + _analogQueueSize[pin]) % 128;
        _analogQueue[pin][tail] = value;
        _analogQueueSize[pin]++;
    }
}
static uint8_t _digitalValues[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
void pinMode(uint8_t pin, uint8_t mode) {}
void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 16) _digitalValues[pin] = val; }
int digitalRead(uint8_t pin) { return (pin < 16) ? _digitalValues[pin] : 0; }
void setDigitalRead(uint8_t pin, uint8_t val) { if (pin < 16) _digitalValues[pin] = val; }
void analogReadResolution(int res) {}
void analogSetPinAttenuation(int pin, int atten) {}
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
