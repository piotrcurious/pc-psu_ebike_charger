#ifndef ADAFRUIT_SSD1306_H
#define ADAFRUIT_SSD1306_H
#include "Adafruit_GFX/Adafruit_GFX.h"
#include "Wire/Wire.h"
#include <string>
#include <vector>
#define SSD1306_SWITCHCAPVCC 0x02
#define SSD1306_WHITE 1
class Adafruit_SSD1306 : public Adafruit_GFX {
public:
    Adafruit_SSD1306(int w, int h, TwoWire* w_ptr, int reset_pin) : Adafruit_GFX(w, h) {}
    bool begin(int vcc_state, int i2c_addr) { return true; }
    void display() { _renderedBuffer = _currentBuffer; }
    void clearDisplay() { _currentBuffer.clear(); }
    void setTextSize(int s) {}
    void setTextColor(int c) {}
    void setTextColor(int c, int bg) {}
    void setCursor(int x, int y) {}
    void print(const char* s) { _currentBuffer += s; }
    void print(std::string s) { _currentBuffer += s; }
    void print(float f, int p = 2) { char b[32]; snprintf(b, 32, "%.*f", p, f); _currentBuffer += b; }
    void print(double d, int p = 2) { char b[32]; snprintf(b, 32, "%.*f", p, d); _currentBuffer += b; }
    void print(int i) { _currentBuffer += std::to_string(i); }
    void print(unsigned int i) { _currentBuffer += std::to_string(i); }
    void print(unsigned long i) { _currentBuffer += std::to_string(i); }
    void println(const char* s) { _currentBuffer += s; _currentBuffer += "\n"; }
    void println(float f, int p = 2) { print(f, p); _currentBuffer += "\n"; }
    void println(double d, int p = 2) { print(d, p); _currentBuffer += "\n"; }
    void println(int i) { _currentBuffer += std::to_string(i); _currentBuffer += "\n"; }
    void println(unsigned int i) { _currentBuffer += std::to_string(i); _currentBuffer += "\n"; }
    void println(unsigned long i) { _currentBuffer += std::to_string(i); _currentBuffer += "\n"; }
    void println() { _currentBuffer += "\n"; }

    std::string getBuffer() const { return _renderedBuffer; }
    void drawRect(int x, int y, int w, int h, int c) {}
    void fillRect(int x, int y, int w, int h, int c) {}
    void drawLine(int x1, int y1, int x2, int y2, int c) {}
private:
    std::string _currentBuffer;
    std::string _renderedBuffer;
};
#endif
