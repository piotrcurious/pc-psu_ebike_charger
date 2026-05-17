#ifndef CHARGER_H
#define CHARGER_H

#include <Arduino.h>
#include "config.h"
#include "Filter.h"
#include "Storage.h"

enum ChargerState_t { IDLE, CHARGING, PAUSED_CHECK_VOLTAGE, CHARGED_COMPLETE, ERROR_STATE };
enum ErrorType_t { NO_ERROR, OVERCURRENT, TIMEOUT, DISCONNECTED, OVERTEMP, CAPACITY_LIMIT };

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
    float ah() const { return _integratedAh; }
    float wh() const { return _integratedWh; }
    float lifetimeAh() const { return _lifetimeAh + _integratedAh; }
    float lifetimeWh() const { return _lifetimeWh + _integratedWh; }
    unsigned long chargeTime() const;
    int pwmDuty() const { return _currentPwmDuty; }
    int fanDuty() const { return _fanDuty; }
    bool isPsuOn() const;
    float batteryInternalResistance() const { return _batteryInternalResistance; }
    int currentOffsetRaw() const { return _currentOffsetRaw; }

    // Setpoints
    float targetVoltage() const { return _targetVoltage; }
    float currentLimit() const { return _currentLimit; }

private:
    void readSensors();
    float calculateTemp(int rawADC);
    void setPwm(int duty);
    void setPsu(bool on);
    void setFan(int duty);
    void updateFan();
    void handleError(ErrorType_t type, const char* msg);
    void handleCharging(unsigned long now, float dt);
    void updateIntegrators(float dt);
    int analogReadAveraged(int pin);

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

    float _integratedAh;
    float _integratedWh;
    float _lifetimeAh;
    float _lifetimeWh;
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
