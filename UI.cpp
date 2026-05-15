#include "UI.h"

UI::UI(Adafruit_SSD1306& display, Charger& charger) :
    _display(display),
    _charger(charger),
    _currentScreen(0),
    _lastButtonState(HIGH),
    _lastButtonChangeTime(0),
    _historyIndex(0),
    _lastGraphUpdateTime(0)
{
    for(int i=0; i<GRAPH_BUFFER_SIZE; i++) _voltageHistory[i] = 0.0f;
}

void UI::setup() {
    pinMode(UI_BUTTON_PIN, INPUT_PULLUP);
    if (!_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
    } else {
        _display.clearDisplay();
        _display.setTextColor(SSD1306_WHITE);
        _display.println("Ready");
        _display.display();
    }
}

void UI::update() {
    handleButton();

    unsigned long now = millis();
    if (now - _lastGraphUpdateTime >= GRAPH_UPDATE_INTERVAL_MS) {
        _lastGraphUpdateTime = now;
        _voltageHistory[_historyIndex] = _charger.vBat();
        _historyIndex = (_historyIndex + 1) % GRAPH_BUFFER_SIZE;
    }

    drawScreen();
}

void UI::handleButton() {
    bool btnState = digitalRead(UI_BUTTON_PIN);
    unsigned long now = millis();

    if (btnState != _lastButtonState) {
        if (now - _lastButtonChangeTime > 50) {
            if (btnState == LOW) {
                _currentScreen = (_currentScreen + 1) % 4;
            }
            _lastButtonState = btnState;
            _lastButtonChangeTime = now;
        }
    }
}

void UI::drawScreen() {
    _display.clearDisplay();
    _display.setTextColor(SSD1306_WHITE);

    if (_charger.state() == ERROR_STATE) {
        drawError();
    } else {
        drawHeader();
        switch (_currentScreen) {
            case 0: drawStatus(); break;
            case 1: drawLive(); break;
            case 2: drawGraph(); break;
            case 3: drawSummary(); break;
        }
    }
    _display.display();
}

void UI::drawHeader() {
    _display.setTextSize(1);
    _display.setCursor(0, 0);
    switch (_charger.state()) {
        case IDLE: _display.print("IDLE"); break;
        case CHARGING: _display.print("CHARGING"); break;
        case PAUSED_CHECK_VOLTAGE: _display.print("PAUSED"); break;
        case CHARGED_COMPLETE: _display.print("CHARGED!"); break;
        default: break;
    }

    if (_charger.state() != IDLE) {
        unsigned long elapsed = _charger.chargeTime();
        int hours = elapsed / 3600000;
        int mins = (elapsed % 3600000) / 60000;
        _display.setCursor(70, 0);
        if (hours < 10) _display.print("0");
        _display.print(hours); _display.print(":");
        if (mins < 10) _display.print("0");
        _display.print(mins);
    }
}

void UI::drawStatus() {
    _display.setTextSize(2);
    _display.setCursor(0, 12);
    _display.print(_charger.vBat(), 1); _display.print("V");

    _display.setTextSize(1);
    _display.setCursor(70, 12);
    _display.print(_charger.iChg(), 2); _display.print("A");
    _display.setCursor(70, 22);
    _display.print(_charger.temp(), 0); _display.print("C");

    _display.setCursor(0, 32);
    if (_charger.isPsuOn()) {
        _display.print("PSU: ON");
    } else {
        _display.print("PSU: OFF");
    }

    float targetV = _charger.targetVoltage();
    float minV = targetV - 5.0f;
    if (minV < 15.0f) minV = 15.0f;
    int progress = (int)((_charger.vBat() - minV) / (targetV - minV) * 100.0f);
    progress = constrain(progress, 0, 100);
    _display.drawRect(0, 44, 102, 10, SSD1306_WHITE);
    _display.fillRect(2, 46, (int)(progress * 0.98f), 6, SSD1306_WHITE);
    _display.setCursor(105, 46);
    _display.print(progress); _display.print("%");

    _display.setCursor(0, 56);
    _display.print("PWM: "); _display.print(_charger.pwmDuty());
    _display.print("/"); _display.print(MAX_PWM_DUTY);
}

void UI::drawLive() {
    _display.setTextSize(2);
    _display.setCursor(0, 15);
    _display.print(_charger.vBat(), 2); _display.println(" V");
    _display.setCursor(0, 35);
    _display.print(_charger.iChg(), 3); _display.println(" A");
    _display.setTextSize(1);
    _display.setCursor(0, 55);
    _display.print(_charger.temp(), 1); _display.print("C ");
    _display.print(_charger.vBat() * _charger.iChg(), 1); _display.print("W");
    _display.setCursor(70, 55);
    _display.print("D:"); _display.print((float)_charger.pwmDuty()/MAX_PWM_DUTY*100.0f, 1); _display.print("%");
}

void UI::drawGraph() {
    _display.drawRect(0, 12, 128, 40, SSD1306_WHITE);

    float minV = 100.0f;
    float maxV = 0.0f;
    bool hasData = false;

    for (int i = 0; i < GRAPH_BUFFER_SIZE; i++) {
        if (_voltageHistory[i] > 1.0f) {
            if (_voltageHistory[i] < minV) minV = _voltageHistory[i];
            if (_voltageHistory[i] > maxV) maxV = _voltageHistory[i];
            hasData = true;
        }
    }

    if (!hasData || (maxV - minV) < 0.1f) {
        minV = _charger.targetVoltage() - 5.0f;
        maxV = _charger.targetVoltage();
    } else {
        minV -= 0.2f;
        maxV += 0.2f;
    }

    for (int i = 0; i < GRAPH_BUFFER_SIZE - 1; i++) {
        int idx1 = (_historyIndex + i) % GRAPH_BUFFER_SIZE;
        int idx2 = (idx1 + 1) % GRAPH_BUFFER_SIZE;
        if (_voltageHistory[idx1] > 1.0f && _voltageHistory[idx2] > 1.0f) {
            int y1 = 51 - (int)((_voltageHistory[idx1] - minV) / (maxV - minV) * 38.0f);
            int y2 = 51 - (int)((_voltageHistory[idx2] - minV) / (maxV - minV) * 38.0f);
            y1 = constrain(y1, 13, 51); y2 = constrain(y2, 13, 51);
            _display.drawLine(i*2, y1, (i+1)*2, y2, SSD1306_WHITE);
        }
    }
    _display.setCursor(0, 54);
    _display.print(minV, 1); _display.print("-"); _display.print(maxV, 1); _display.print("V");
}

void UI::drawSummary() {
    _display.setCursor(0, 15);
    _display.println("Energy Log:");
    _display.print("Ah: "); _display.println(_charger.ah(), 3);
    _display.print("Wh: "); _display.println(_charger.wh(), 2);
    _display.print("Rbat: "); _display.print(_charger.batteryInternalResistance(), 3); _display.println(" ohm");
    _display.print("Time: "); _display.print(_charger.chargeTime()/60000); _display.println(" mins");
}

void UI::drawError() {
    _display.setCursor(0, 0);
    _display.print("ERROR!");
    _display.setCursor(0, 16);
    switch(_charger.lastError()) {
        case OVERCURRENT: _display.println("OVERCURRENT"); break;
        case TIMEOUT: _display.println("TIMEOUT"); break;
        case DISCONNECTED: _display.println("DISCONNECTED"); break;
        case OVERTEMP: _display.println("OVERTEMP"); break;
        default: _display.println("UNKNOWN ERROR"); break;
    }
    _display.println("\nReset required.");
}
