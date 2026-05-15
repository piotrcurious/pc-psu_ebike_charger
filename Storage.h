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

    void save(float ah, float wh) {
        _lifetimeAh = ah;
        _lifetimeWh = wh;
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        _prefs.putFloat("ah", _lifetimeAh);
        _prefs.putFloat("wh", _lifetimeWh);
#endif
    }

    float lifetimeAh() const { return _lifetimeAh; }
    float lifetimeWh() const { return _lifetimeWh; }

private:
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    Preferences _prefs;
#endif
    float _lifetimeAh;
    float _lifetimeWh;
};

#endif
