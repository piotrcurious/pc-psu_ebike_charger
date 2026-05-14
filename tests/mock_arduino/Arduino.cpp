#include "Arduino.h"
#include <vector>

SerialMock Serial;

static unsigned long _millis = 0;

unsigned long millis() {
    return _millis;
}

void delay(unsigned long ms) {
    _millis += ms;
}

void delayMicroseconds(unsigned int us) {
}

void set_millis(unsigned long ms) {
    _millis = ms;
}

void advance_millis(unsigned long ms) {
    _millis += ms;
}

static int _analogValues[16] = {0};
int analogRead(uint8_t pin) {
    if (pin < 16) return _analogValues[pin];
    return 0;
}

void setAnalogRead(uint8_t pin, int value) {
    if (pin < 16) _analogValues[pin] = value;
}

void pinMode(uint8_t pin, uint8_t mode) {}
void digitalWrite(uint8_t pin, uint8_t val) {}
int digitalRead(uint8_t pin) { return 0; }
void analogReadResolution(int res) {}
void analogSetPinAttenuation(int pin, int atten) {}

static uint32_t _pwmValues[16] = {0};
void ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits) {}
void ledcAttachPin(uint8_t pin, uint8_t channel) {}
void ledcWrite(uint8_t channel, uint32_t duty) {
    if (channel < 16) _pwmValues[channel] = duty;
}

uint32_t getPwmWrite(uint8_t channel) {
    if (channel < 16) return _pwmValues[channel];
    return 0;
}

#include "Wire/Wire.h"
TwoWire Wire;
