#ifndef ARDUINO_H
#define ARDUINO_H
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <string>
typedef uint8_t byte;
typedef bool boolean;
#define HIGH 0x1
#define LOW  0x0
#define INPUT 0x01
#define OUTPUT 0x02
#define INPUT_PULLUP 0x03
#define PI 3.1415926535897932384626433832795
#define ESP32 1
#ifdef __cplusplus
extern "C" {
#endif
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogReadResolution(int res);
void analogSetPinAttenuation(int pin, int atten);
unsigned long millis();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);
#ifdef __cplusplus
}
#endif
#define ADC_ATTEN_DB_11 3
#define ADC_11db 3
#ifdef __cplusplus
class SerialMock {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) { printf("%s", s); }
    void print(std::string s) { printf("%s", s.c_str()); }
    void print(float f, int p = 2) { printf("%.*f", p, f); }
    void print(double d, int p = 2) { printf("%.*f", p, d); }
    void print(int i) { printf("%d", i); }
    void print(unsigned int i) { printf("%u", i); }
    void print(unsigned long i) { printf("%lu", i); }
    void println(const char* s) { printf("%s\n", s); }
    void println(std::string s) { printf("%s\n", s.c_str()); }
    void println(float f, int p = 2) { printf("%.*f\n", p, f); }
    void println(double d, int p = 2) { printf("%.*f\n", p, d); }
    void println(int i) { printf("%d\n", i); }
    void println(unsigned int i) { printf("%u\n", i); }
    void println(unsigned long i) { printf("%lu\n", i); }
    void println() { printf("\n"); }
    int available() { return 0; }
    int read() { return -1; }
};
extern SerialMock Serial;
#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif
using std::max;
using std::min;
#endif
#define F(x) x
#endif
