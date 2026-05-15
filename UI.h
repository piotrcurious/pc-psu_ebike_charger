#ifndef UI_H
#define UI_H

#include <Adafruit_SSD1306.h>
#include "Charger.h"

class UI {
public:
    UI(Adafruit_SSD1306& display, Charger& charger);
    void setup();
    void update();

private:
    void handleButton();
    void drawScreen();
    void drawHeader();
    void drawStatus();
    void drawLive();
    void drawGraph();
    void drawSummary();
    void drawError();

    Adafruit_SSD1306& _display;
    Charger& _charger;

    int _currentScreen;
    bool _lastButtonState;
    unsigned long _lastButtonChangeTime;

    float _voltageHistory[GRAPH_BUFFER_SIZE];
    int _historyIndex;
    unsigned long _lastGraphUpdateTime;
    unsigned long _lastRefreshTime;
};

#endif
