#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <Preferences.h>
#endif

class ChargerStorage {
public:
    ChargerStorage() : _lifetimeAh(0), _lifetimeWh(0) {}

    void begin() {
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        _prefs.begin("charger", false);
        _lifetimeAh = _prefs.getFloat("ah", 0);
        _lifetimeWh = _prefs.getFloat("wh", 0);
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

    double lifetimeAh() const { return _lifetimeAh; }
    double lifetimeWh() const { return _lifetimeWh; }

private:
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences _prefs;
#endif
    double _lifetimeAh;
    double _lifetimeWh;
};

#endif
