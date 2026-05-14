#ifndef ADAFRUIT_SSD1306_H
#define ADAFRUIT_SSD1306_H
#include "Adafruit_GFX/Adafruit_GFX.h"
#include "Wire/Wire.h"
#define SSD1306_SWITCHCAPVCC 0x02
#define SSD1306_WHITE 1
class Adafruit_SSD1306 : public Adafruit_GFX {
public:
    Adafruit_SSD1306(int w, int h, TwoWire* w_ptr, int reset_pin) : Adafruit_GFX(w, h) {}
    bool begin(int vcc_state, int i2c_addr) { return true; }
    void display() {}
    void clearDisplay() {}
    void setTextSize(int s) {}
    void setTextColor(int c) {}
    void setCursor(int x, int y) {}
    void print(const char* s) {}
    void print(float f, int p = 2) {}
    void print(int i) {}
    void println(const char* s) {}
    void println(float f, int p = 2) {}
    void println(int i) {}
    void println() {}
    void drawRect(int x, int y, int w, int h, int c) {}
    void fillRect(int x, int y, int w, int h, int c) {}
};
#endif
