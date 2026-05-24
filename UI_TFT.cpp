#include "UI_TFT.h"

UI_TFT::UI_TFT(LGFX& tft, Charger& charger) :
    _tft(tft),
    _charger(charger),
    _currentScreen(0),
    _lastButtonState(HIGH),
    _lastButtonChangeTime(0),
    _historyIndex(0),
    _lastGraphUpdateTime(0),
    _lastRefreshTime(0)
{
    for(int i=0; i<GRAPH_BUFFER_SIZE; i++) _voltageHistory[i] = 0.0f;
}

void UI_TFT::setup() {
    pinMode(UI_BUTTON_PIN, INPUT_PULLUP);
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);

    _tft.init();
    _tft.setRotation(1);
    _tft.fillScreen(TFT_BLACK);
    _tft.setTextColor(TFT_WHITE);
    _tft.setTextSize(2);
    _tft.println("Charger System Ready");
}

void UI_TFT::update() {
    handleButton();

    unsigned long now = millis();
    if (now - _lastGraphUpdateTime >= GRAPH_UPDATE_INTERVAL_MS) {
        _lastGraphUpdateTime = now;
        _voltageHistory[_historyIndex] = _charger.vBat();
        _historyIndex = (_historyIndex + 1) % GRAPH_BUFFER_SIZE;
    }

    if (now - _lastRefreshTime >= UI_REFRESH_INTERVAL_MS) {
        _lastRefreshTime = now;
        drawScreen();
    }
}

void UI_TFT::handleButton() {
    bool btnState = digitalRead(UI_BUTTON_PIN);
    unsigned long now = millis();

    if (btnState != _lastButtonState) {
        if (now - _lastButtonChangeTime > 50) {
            if (btnState == HIGH) { // Release
                unsigned long pressDuration = now - _lastButtonChangeTime;
                if (pressDuration > 2000) {
                    if (_currentScreen == 5) _charger.calibrateCurrentSensor();
                    else if (_charger.state() == IDLE || _charger.state() == CHARGED_COMPLETE || _charger.state() == ERROR_STATE) {
                        _charger.resetSession();
                    }
                } else {
                    _currentScreen = (_currentScreen + 1) % 6;
                    _tft.fillScreen(TFT_BLACK); // Clear on screen change
                }
            }
            _lastButtonState = btnState;
            _lastButtonChangeTime = now;
        }
    }
}

uint32_t UI_TFT::getStatusColor() {
    switch (_charger.state()) {
        case IDLE: return TFT_LIGHTGRAY;
        case CHARGING: return TFT_GREEN;
        case PAUSED_CHECK_VOLTAGE: return TFT_YELLOW;
        case CHARGED_COMPLETE: return TFT_CYAN;
        case ERROR_STATE: return TFT_RED;
        default: return TFT_WHITE;
    }
}

void UI_TFT::drawScreen() {
    _tft.startWrite();
    if (_charger.state() == ERROR_STATE) {
        drawError();
    } else {
        drawHeader();
        switch (_currentScreen) {
            case 0: drawStatus(); break;
            case 1: drawLive(); break;
            case 2: drawGraph(); break;
            case 3: drawSummary(); break;
            case 4: drawLifetime(); break;
            case 5: drawDiagnostics(); break;
        }
    }
    _tft.endWrite();
}

void UI_TFT::drawHeader() {
    _tft.fillRect(0, 0, 320, 30, TFT_DARKGREY);
    _tft.setTextColor(TFT_WHITE);
    _tft.setTextSize(2);
    _tft.setCursor(10, 5);

    switch (_charger.state()) {
        case IDLE: _tft.print("IDLE"); break;
        case CHARGING: _tft.print("CHARGING"); break;
        case PAUSED_CHECK_VOLTAGE: _tft.print("CHECKING VOLTAGE"); break;
        case CHARGED_COMPLETE: _tft.print("CHARGED COMPLETE"); break;
        default: break;
    }

    unsigned long elapsed = _charger.chargeTime();
    int hours = elapsed / 3600000;
    int mins = (elapsed % 3600000) / 60000;
    _tft.setCursor(240, 5);
    _tft.printf("%02d:%02d", hours, mins);
}

void UI_TFT::drawStatus() {
    uint32_t color = getStatusColor();

    // Voltage
    _tft.setTextColor(color);
    _tft.setTextSize(5);
    _tft.setCursor(20, 50);
    _tft.printf("%.1fV", _charger.vBat());

    // Current
    _tft.setTextSize(3);
    _tft.setCursor(200, 55);
    _tft.printf("%.2fA", _charger.iChg());

    // Temperature & Fan
    _tft.setTextSize(2);
    _tft.setTextColor(TFT_WHITE);
    _tft.setCursor(20, 110);
    _tft.printf("Temp: %.1f C", _charger.temp());
    if (_charger.fanDuty() > 0) {
        _tft.printf(" | Fan: %d%%", _charger.fanDuty() * 100 / MAX_FAN_DUTY);
    }

    // Progress Bar
    float targetV = _charger.targetVoltage();
    float minV = 20.0f;
    int progress = (int)((_charger.vBat() - minV) / (targetV - minV) * 100.0f);
    progress = constrain(progress, 0, 100);

    _tft.drawRect(20, 150, 280, 30, TFT_WHITE);
    _tft.fillRect(22, 152, (int)(progress * 2.76f), 26, color);

    _tft.setCursor(140, 190);
    _tft.setTextSize(2);
    _tft.printf("%d%%", progress);

    _tft.setCursor(20, 220);
    _tft.setTextSize(1);
    _tft.printf("PWM: %d/%d | Target: %.1fV Lim: %.1fA", _charger.pwmDuty(), MAX_PWM_DUTY, targetV, _charger.currentLimit());
}

void UI_TFT::drawLive() {
    _tft.setTextSize(3);
    _tft.setTextColor(TFT_YELLOW);
    _tft.setCursor(10, 40);
    _tft.printf("V_bat:   %.2f V", _charger.vBat());

    _tft.setTextColor(TFT_ORANGE);
    _tft.setCursor(10, 80);
    _tft.printf("I_chg:   %.3f A", _charger.iChg());

    _tft.setTextColor(TFT_GREEN);
    _tft.setCursor(10, 120);
    _tft.printf("Power:   %.1f W", _charger.vBat() * _charger.iChg());

    _tft.setTextColor(TFT_WHITE);
    _tft.setCursor(10, 160);
    _tft.printf("Energy:  %.3f Ah", _charger.ah());
    _tft.setCursor(10, 200);
    _tft.printf("         %.1f Wh", _charger.wh());
}

void UI_TFT::drawGraph() {
    int gx = 40;
    int gy = 40;
    int gw = 260;
    int gh = 160;

    _tft.drawRect(gx, gy, gw, gh, TFT_WHITE);

    float minV = 100.0f, maxV = 0.0f;
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
        maxV = _charger.targetVoltage() + 1.0f;
    } else {
        minV -= 0.5f; maxV += 0.5f;
    }

    // Y-axis labels
    _tft.setTextSize(1);
    _tft.setTextColor(TFT_WHITE);
    _tft.setCursor(gx - 35, gy); _tft.printf("%.1f", maxV);
    _tft.setCursor(gx - 35, gy + gh - 10); _tft.printf("%.1f", minV);

    // Target Line
    float targetV = _charger.targetVoltage();
    if (targetV > minV && targetV < maxV) {
        int targetY = gy + gh - (int)((targetV - minV) / (maxV - minV) * gh);
        for(int x = gx; x < gx+gw; x+=8) _tft.drawFastHLine(x, targetY, 4, TFT_BLUE);
    }

    // Plot History
    for (int i = 0; i < GRAPH_BUFFER_SIZE - 1; i++) {
        int idx1 = (_historyIndex + i) % GRAPH_BUFFER_SIZE;
        int idx2 = (idx1 + 1) % GRAPH_BUFFER_SIZE;
        if (_voltageHistory[idx1] > 1.0f && _voltageHistory[idx2] > 1.0f) {
            int y1 = gy + gh - (int)((_voltageHistory[idx1] - minV) / (maxV - minV) * gh);
            int y2 = gy + gh - (int)((_voltageHistory[idx2] - minV) / (maxV - minV) * gh);
            float stepX = (float)gw / GRAPH_BUFFER_SIZE;
            _tft.drawLine(gx + i*stepX, y1, gx + (i+1)*stepX, y2, TFT_YELLOW);
        }
    }
}

void UI_TFT::drawSummary() {
    _tft.setTextSize(3);
    _tft.setTextColor(TFT_CYAN);
    _tft.setCursor(10, 40);
    _tft.println("Session Log:");

    _tft.setTextSize(2);
    _tft.setTextColor(TFT_WHITE);
    _tft.setCursor(10, 80);
    _tft.printf("Delivered: %.3f Ah", _charger.ah());
    _tft.setCursor(10, 110);
    _tft.printf("           %.2f Wh", _charger.wh());

    _tft.setCursor(10, 140);
    _tft.printf("Resistance: %.3f Ohm", _charger.batteryInternalResistance());

    _tft.setCursor(10, 170);
    _tft.printf("Duration:   %lu mins", _charger.chargeTime()/60000);
}

void UI_TFT::drawLifetime() {
    _tft.setTextSize(3);
    _tft.setTextColor(TFT_MAGENTA);
    _tft.setCursor(10, 40);
    _tft.println("Lifetime Stats:");

    _tft.setTextSize(2);
    _tft.setTextColor(TFT_WHITE);
    _tft.setCursor(10, 90);
    _tft.printf("Total Ah: %.1f", _charger.lifetimeAh());
    _tft.setCursor(10, 120);
    _tft.printf("Total Wh: %.0f", _charger.lifetimeWh());
}

void UI_TFT::drawDiagnostics() {
    _tft.setTextSize(2);
    _tft.setTextColor(TFT_GREEN);
    _tft.setCursor(10, 40);
    _tft.println("System Diagnostics:");

    _tft.setTextColor(TFT_WHITE);
    _tft.setCursor(10, 70);
    _tft.printf("Est Voc:  %.2f V", _charger.vBatOC());
    _tft.setCursor(10, 95);
    _tft.printf("I_offset: %d mV", _charger.currentOffsetRaw());
    _tft.setCursor(10, 120);
    _tft.printf("Target:   %.2f V", _charger.targetVoltage());
    _tft.setCursor(10, 145);
    _tft.printf("Limit:    %.2f A", _charger.currentLimit());
    _tft.setCursor(10, 170);
    _tft.printf("Soft Lim: %.2f A", _charger.softStartLimit());
    _tft.setCursor(10, 195);
    _tft.printf("R_bat:    %.3f Ohm", _charger.batteryInternalResistance());

    _tft.setTextColor(TFT_YELLOW);
    _tft.setCursor(10, 220);
    _tft.setTextSize(1);
    _tft.println("Hold Button to Calibrate Current Sensor");
}

void UI_TFT::drawError() {
    _tft.fillScreen(TFT_RED);
    _tft.setTextColor(TFT_WHITE);
    _tft.setTextSize(4);
    _tft.setCursor(10, 40);
    _tft.println("SYSTEM HALT");

    _tft.setTextSize(3);
    _tft.setCursor(10, 100);
    switch(_charger.lastError()) {
        case OVERCURRENT: _tft.println("OVERCURRENT"); break;
        case TIMEOUT:     _tft.println("TIMEOUT"); break;
        case DISCONNECTED: _tft.println("DISCONNECTED"); break;
        case OVERTEMP:    _tft.println("OVERTEMP"); break;
        case CAPACITY_LIMIT: _tft.println("CAPACITY LIMIT"); break;
        case SENSOR_FAULT: _tft.println("SENSOR FAULT"); break;
        default:          _tft.println("UNKNOWN ERROR"); break;
    }

    _tft.setTextSize(2);
    _tft.setCursor(10, 200);
    _tft.println("Safety shutdown engaged.");
    _tft.println("Manual reset required.");
}
