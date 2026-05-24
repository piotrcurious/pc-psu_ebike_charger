#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <Preferences.h>
#endif

class ChargerStorage {
public:
    ChargerStorage() : _lifetimeAh(0), _lifetimeWh(0), _currentOffsetRaw(2048) {}

    void begin() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        _prefs.begin("charger", false);
        _lifetimeAh = _prefs.getFloat("ah", 0);
        _lifetimeWh = _prefs.getFloat("wh", 0);
        _currentOffsetRaw = _prefs.getInt("offset", 2048);
#endif
    }

    void save(double ah, double wh) {
        _lifetimeAh = ah;
        _lifetimeWh = wh;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        _prefs.putFloat("ah", (float)_lifetimeAh);
        _prefs.putFloat("wh", (float)_lifetimeWh);
#endif
    }

    void saveOffset(int offset) {
        _currentOffsetRaw = offset;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        _prefs.putInt("offset", _currentOffsetRaw);
#endif
    }

    double lifetimeAh() const { return _lifetimeAh; }
    double lifetimeWh() const { return _lifetimeWh; }
    int currentOffsetRaw() const { return _currentOffsetRaw; }

private:
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences _prefs;
#endif
    double _lifetimeAh;
    double _lifetimeWh;
    int _currentOffsetRaw;
};

#endif
