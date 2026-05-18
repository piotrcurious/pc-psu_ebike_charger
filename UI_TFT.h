#ifndef UI_TFT_H
#define UI_TFT_H

#include "LGFX_Config.h"
#include "Charger.h"

class UI_TFT {
public:
    UI_TFT(LGFX& tft, Charger& charger);
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
    void drawLifetime();
    void drawDiagnostics();
    void drawError();

    LGFX& _tft;
    Charger& _charger;

    int _currentScreen;
    bool _lastButtonState;
    unsigned long _lastButtonChangeTime;

    float _voltageHistory[GRAPH_BUFFER_SIZE];
    int _historyIndex;
    unsigned long _lastGraphUpdateTime;
    unsigned long _lastRefreshTime;

    uint32_t getStatusColor();
};

#endif
