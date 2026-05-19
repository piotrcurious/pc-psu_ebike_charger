#ifndef CHARGER_H
#define CHARGER_H

#include <Arduino.h>
#if defined(ESP32_WROOM_TFT)
#include "config_esp32.h"
#else
#include "config.h"
#endif
#include "Filter.h"
#include "Storage.h"

enum ChargerState_t { IDLE, CHARGING, PAUSED_CHECK_VOLTAGE, CHARGED_COMPLETE, ERROR_STATE };
enum ErrorType_t { NO_ERROR, OVERCURRENT, TIMEOUT, DISCONNECTED, OVERTEMP, CAPACITY_LIMIT, SENSOR_FAULT, OVERVOLTAGE };

class Charger {
public:
    Charger();
    void setup();
    void update(float dt);
    void calibrateCurrentSensor();
    void reset();
    void resetSession();

    // Data access
    ChargerState_t state() const { return _state; }
    ErrorType_t lastError() const { return _lastError; }
    float vBat() const { return _vBatFilter.value(); }
    float iChg() const { return _iChgFilter.value(); }
    float temp() const { return _tempFilter.value(); }
    double ah() const { return _integratedAh; }
    double wh() const { return _integratedWh; }
    double lifetimeAh() const { return _lifetimeAh + _integratedAh; }
    double lifetimeWh() const { return _lifetimeWh + _integratedWh; }
    unsigned long chargeTime() const;
    int pwmDuty() const { return _currentPwmDuty; }
    int fanDuty() const { return _fanDuty; }
    bool isPsuOn() const;
    float batteryInternalResistance() const { return _batteryInternalResistance; }
    float vBatOC() const;
    int currentOffsetRaw() const { return _currentOffsetRaw; }

    // Setpoints
    float targetVoltage() const { return _targetVoltage; }
    float currentLimit() const { return _currentLimit; }

private:
    void readSensors();
    float calculateTemp(uint32_t mv);
    void setPwm(int duty);
    void setPsu(bool on);
    void setFan(int duty);
    void updateFan();
    void handleError(ErrorType_t type, const char* msg);
    void handleCharging(unsigned long now, float dt);
    void updateIntegrators(float dt);
    uint32_t analogReadMilliVoltsAveraged(int pin);

    ChargerState_t _state;
    ErrorType_t _lastError;

    EMAFilter<float> _vBatFilter;
    EMAFilter<float> _iChgFilter;
    EMAFilter<float> _tempFilter;

    float _batteryVoltage;
    float _chargingCurrent;
    float _temperature;
    float _targetVoltage;
    float _currentLimit;
    float _softStartLimit;
    int _currentPwmDuty;
    int _fanDuty;

    int _currentOffsetRaw;

    double _integratedAh;
    double _integratedWh;
    double _lifetimeAh;
    double _lifetimeWh;
    float _batteryInternalResistance;

    unsigned long _chargeStartTime;
    unsigned long _chargeEndTime;
    unsigned long _lastChargeCheckTime;
    unsigned long _pausedStartTime;
    unsigned long _lastPsuOffTime;
    unsigned long _lastDiagTime;
    unsigned long _lastSaveTime;
    unsigned long _fullConditionStartTime;

    float _lastIFilteredForDisconnect;
    float _vLoadedAtPause;
    float _iLoadedAtPause;

    ChargerStorage _storage;
};

#endif
