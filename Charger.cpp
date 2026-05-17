#include "Charger.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#include <esp_task_wdt.h>
#endif

Charger::Charger() :
    _state(IDLE),
    _lastError(NO_ERROR),
    _vBatFilter(FILTER_ALPHA_V),
    _iChgFilter(FILTER_ALPHA_I),
    _tempFilter(FILTER_ALPHA_T),
    _batteryVoltage(0),
    _chargingCurrent(0),
    _temperature(25.0),
    _targetVoltage(29.0),
    _currentLimit(1.0),
    _softStartLimit(0.05),
    _currentPwmDuty(0),
    _fanDuty(0),
    _currentOffsetRaw(2048),
    _integratedAh(0),
    _integratedWh(0),
    _lifetimeAh(0),
    _lifetimeWh(0),
    _batteryInternalResistance(0),
    _chargeStartTime(0),
    _chargeEndTime(0),
    _lastChargeCheckTime(0),
    _pausedStartTime(0),
    _lastPsuOffTime(0),
    _lastDiagTime(0),
    _lastSaveTime(0),
    _fullConditionStartTime(0),
    _lastIFilteredForDisconnect(0),
    _vLoadedAtPause(0),
    _iLoadedAtPause(0)
{}

void Charger::setup() {
    pinMode(ATX_PS_ON_PIN, OUTPUT);
    setPsu(false);

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_OUT_PIN, PWM_CHANNEL);
    setPwm(0);

    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RES);
    ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
    setFan(0);

    #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
        analogReadResolution(12);
        analogSetPinAttenuation(BAT_VOLTAGE_SENSE_PIN, ADC_ATTEN_DB_11);
        analogSetPinAttenuation(CURRENT_SENSE_AMP_PIN, ADC_ATTEN_DB_11);
        analogSetPinAttenuation(DESIRED_VOLTAGE_SET_PIN, ADC_ATTEN_DB_11);
        analogSetPinAttenuation(DESIRED_CURRENT_SET_PIN, ADC_ATTEN_DB_11);
        analogSetPinAttenuation(TEMP_SENSE_PIN, ADC_ATTEN_DB_11);

        esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
        esp_task_wdt_add(NULL);
    #endif

    _storage.begin();
    _lifetimeAh = _storage.lifetimeAh();
    _lifetimeWh = _storage.lifetimeWh();

    calibrateCurrentSensor();
    readSensors();
    _lastChargeCheckTime = millis();
    _lastSaveTime = millis();
}

void Charger::calibrateCurrentSensor() {
    setPwm(0);
    setPsu(false);
    delay(1000);
    long sum = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        sum += analogRead(CURRENT_SENSE_AMP_PIN);
        delay(2);
    }
    _currentOffsetRaw = (int)(sum / CALIBRATION_SAMPLES);
    Serial.print("ACS712 Calibrated. Offset: "); Serial.println(_currentOffsetRaw);
}

void Charger::reset() {
    _state = IDLE;
    _lastError = NO_ERROR;
    _vBatFilter.reset();
    _iChgFilter.reset();
    _tempFilter.reset();
    resetSession();
    Serial.println("Charger Reset");
}

void Charger::resetSession() {
    _integratedAh = 0;
    _integratedWh = 0;
    _batteryInternalResistance = 0;
    _fullConditionStartTime = 0;
    _lastIFilteredForDisconnect = 0;
    _chargeStartTime = millis();
    Serial.println("Session Reset");
}

void Charger::update(float dt) {
    #if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
    esp_task_wdt_reset();
    #endif

    unsigned long now = millis();
    static float accumulatedDt = 0;
    accumulatedDt += dt;

    readSensors();
    updateIntegrators(dt);
    updateFan();

    if (temp() > MAX_ALLOWED_TEMP) {
        handleError(OVERTEMP, "Over temperature!");
    }

    if (_integratedAh > MAX_CHARGE_AH_LIMIT) {
        handleError(CAPACITY_LIMIT, "Charge capacity limit reached");
    }

    if (now - _lastSaveTime >= STATS_SAVE_INTERVAL_MS) {
        static float lastSavedAh = 0;
        float currentAh = _lifetimeAh + _integratedAh;
        // Only save if Ah changed significantly (more than 0.1Ah) or first save of the hour
        if (abs(currentAh - lastSavedAh) > 0.1f || (now - _lastSaveTime > 3600000UL)) {
            _lastSaveTime = now;
            lastSavedAh = currentAh;
            _storage.save(currentAh, _lifetimeWh + _integratedWh);
            Serial.println("Lifetime stats saved to Flash");
        }
    }

    if (_state != PAUSED_CHECK_VOLTAGE && now - _lastDiagTime >= 5000) {
        _lastDiagTime = now;
        Serial.print("DIAG: State="); Serial.print((int)_state);
        Serial.print(" Vbat="); Serial.print(vBat(), 2);
        Serial.print(" Ichg="); Serial.print(iChg(), 3);
        Serial.print(" Temp="); Serial.print(temp(), 1);
        Serial.print(" Fan="); Serial.print(_fanDuty);
        Serial.print(" Ah="); Serial.print(_integratedAh, 4);
        Serial.print(" LAh="); Serial.print(lifetimeAh(), 2);
        Serial.print(" PSU="); Serial.println(isPsuOn() ? "ON" : "OFF");
    }

    if (_state != PAUSED_CHECK_VOLTAGE && accumulatedDt < 0.02f) return;
    float logicDt = accumulatedDt;
    accumulatedDt = 0;

    float bvFiltered = vBat();

    switch (_state) {
        case IDLE:
            setPwm(0);
            setPsu(false);
            _softStartLimit = 0.05f;
            if (bvFiltered > MIN_BATTERY_VOLTAGE && bvFiltered < (_targetVoltage - 1.0f)) {
                setPsu(true);
                if (isPsuOn()) {
                    _state = CHARGING;
                    _lastChargeCheckTime = now;
                    _chargeStartTime = now;
                    _integratedAh = 0; _integratedWh = 0;
                    _fullConditionStartTime = 0;
                    Serial.println("State: IDLE -> CHARGING");
                }
            }
            break;

        case CHARGING:
            handleCharging(now, logicDt);
            // PSU Health Check: If PSU is on, PWM is high, but no current
            // Increased timeout to 10s to allow for ramp-up
#ifndef UNIT_TEST
            if (now - _chargeStartTime > 10000 && _currentPwmDuty > (MAX_PWM_DUTY / 2) && iChg() < 0.05f) {
                 handleError(DISCONNECTED, "PSU failure or Battery disconnected");
            }
#endif
            break;

        case PAUSED_CHECK_VOLTAGE:
            setPwm(0);
            if (now - _pausedStartTime < PAUSE_SETTLE_MS) return;
            readSensors();
            {
                float vUnloaded = _batteryVoltage;
                if (_iLoadedAtPause > 0.1f) {
                    float rEst = (_vLoadedAtPause - vUnloaded) / _iLoadedAtPause;
                    if (rEst > 0 && rEst < 5.0f) {
                        if (_batteryInternalResistance == 0) _batteryInternalResistance = rEst;
                        else _batteryInternalResistance = (0.2f * rEst) + (0.8f * _batteryInternalResistance);
                    }
                }
                if (vUnloaded >= _targetVoltage - 0.1f) {
                    Serial.println("Final voltage reached - State: CHARGED_COMPLETE");
                    _chargeEndTime = now;
                    setPsu(false);
                    _state = CHARGED_COMPLETE;
                    _storage.save(_lifetimeAh + _integratedAh, _lifetimeWh + _integratedWh);
                    _lifetimeAh += _integratedAh; _lifetimeWh += _integratedWh;
                    _integratedAh = 0; _integratedWh = 0;
                } else if (vUnloaded < MIN_BATTERY_VOLTAGE) {
                    handleError(DISCONNECTED, "Battery disconnected");
                } else {
                    Serial.println("Voltage not reached - State: CHARGING");
                    _state = CHARGING;
                    _lastChargeCheckTime = now;
                }
            }
            break;

        case CHARGED_COMPLETE:
            setPwm(0);
            setPsu(false);
            _softStartLimit = 0.05f;
            if (bvFiltered > MIN_BATTERY_VOLTAGE && bvFiltered <= (_targetVoltage - 0.5f)) {
                setPsu(true);
                if (isPsuOn()) {
                    _state = CHARGING;
                    _lastChargeCheckTime = now;
                    _chargeStartTime = now;
                    Serial.println("Voltage dropped - State: CHARGING");
                }
            } else if (bvFiltered < MIN_BATTERY_VOLTAGE) {
                _state = IDLE;
            }
            break;

        case ERROR_STATE:
            setPwm(0);
            setPsu(false);
            break;
    }
}

void Charger::handleCharging(unsigned long now, float dt) {
    if (now - _chargeStartTime > MAX_CHARGE_TIME_MS) {
        handleError(TIMEOUT, "Charge timeout");
        return;
    }
    setPsu(true);
    if (!isPsuOn()) return;

    if (_softStartLimit < _currentLimit) {
        _softStartLimit += SOFT_START_RAMP_A_PER_S * dt;
        if (_softStartLimit > _currentLimit) _softStartLimit = _currentLimit;
    } else {
        _softStartLimit = _currentLimit;
    }

    float iFiltered = iChg();
    float bvFiltered = vBat();

    if (iFiltered > (_currentLimit * MAX_ALLOWED_CURRENT_MULTIPLIER + 0.5f)) {
        handleError(OVERCURRENT, "Overcurrent");
        return;
    }

    // Sudden disconnect detection: current was significant but dropped to zero while PWM is high
    if (_lastIFilteredForDisconnect > 0.1f && iFiltered < 0.05f && _currentPwmDuty > 30) {
        handleError(DISCONNECTED, "Sudden disconnect");
        return;
    }
    _lastIFilteredForDisconnect = iFiltered;

    int nextPwm = _currentPwmDuty;
    if (iFiltered > (_softStartLimit + CURRENT_DEADBAND)) {
        nextPwm -= PWM_STEP_DOWN_FAST;
    } else if (bvFiltered > (_targetVoltage + VOLTAGE_DEADBAND)) {
        nextPwm -= PWM_STEP_DOWN_SLOW;
    } else if (iFiltered < (_softStartLimit - CURRENT_DEADBAND) &&
               bvFiltered < (_targetVoltage - VOLTAGE_DEADBAND)) {
        nextPwm += PWM_STEP_UP;
    }
    setPwm(nextPwm);

    if (bvFiltered >= (_targetVoltage - 0.1f) && iFiltered < FULL_CHARGE_CURRENT_THRESHOLD) {
        if (_fullConditionStartTime == 0) _fullConditionStartTime = now;
        if (now - _fullConditionStartTime > 10000) {
            Serial.println("Battery full - State: CHARGED_COMPLETE");
            _chargeEndTime = now;
            setPsu(false);
            _state = CHARGED_COMPLETE;
            _fullConditionStartTime = 0;
            _storage.save(_lifetimeAh + _integratedAh, _lifetimeWh + _integratedWh);
            _lifetimeAh += _integratedAh; _lifetimeWh += _integratedWh;
            _integratedAh = 0; _integratedWh = 0;
            return;
        }
    } else {
        _fullConditionStartTime = 0;
    }

    if (now - _lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
        Serial.println("Periodic check - pausing PWM");
        _vLoadedAtPause = bvFiltered;
        _iLoadedAtPause = iFiltered;
        setPwm(0);
        _pausedStartTime = now;
        _state = PAUSED_CHECK_VOLTAGE;
    }
}

void Charger::readSensors() {
    int rawBat = analogReadAveraged(BAT_VOLTAGE_SENSE_PIN);
    int rawI = analogReadAveraged(CURRENT_SENSE_AMP_PIN);
    int rawSetV = analogReadAveraged(DESIRED_VOLTAGE_SET_PIN);
    int rawSetI = analogReadAveraged(DESIRED_CURRENT_SET_PIN);
    int rawTemp = analogReadAveraged(TEMP_SENSE_PIN);

    _batteryVoltage = (float)rawBat * VOLTAGE_SENSE_FACTOR;
    _chargingCurrent = (float)(rawI - _currentOffsetRaw) * CURRENT_RAW_TO_A;
    _temperature = calculateTemp(rawTemp);

    _vBatFilter.update(_batteryVoltage);
    _iChgFilter.update(_chargingCurrent);
    _tempFilter.update(_temperature);

    _targetVoltage = constrain((float)rawSetV * ADC_V_PER_COUNT * (30.0f / 3.3f), 26.0f, 30.0f);
    _currentLimit = constrain((float)rawSetI * ADC_V_PER_COUNT * (5.0f / 3.3f), 0.1f, 5.0f);
}

float Charger::calculateTemp(int rawADC) {
    if (rawADC == 0 || rawADC >= 4095) return 25.0f;
    float voltage = rawADC * ADC_V_PER_COUNT;
    if (voltage >= 3.25f) return 25.0f;
    float resistance = NTC_R_SERIES * (voltage / (3.3f - voltage));
    float steinhart = resistance / NTC_NOMINAL_R;
    steinhart = log(steinhart);
    steinhart /= NTC_BETA;
    steinhart += 1.0f / (NTC_NOMINAL_T + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;
    return steinhart;
}

void Charger::updateIntegrators(float dt) {
    if (_state == CHARGING) {
        float i = iChg();
        if (i > 0) {
            _integratedAh += (i * dt) / 3600.0f;
            _integratedWh += (i * vBat() * dt) / 3600.0f;
        }
    }
}

void Charger::updateFan() {
    float t = temp();
    if (t < FAN_TEMP_MIN) {
        setFan(0);
    } else {
        float duty = (t - FAN_TEMP_MIN) / (FAN_TEMP_MAX - FAN_TEMP_MIN) * MAX_FAN_DUTY;
        setFan((int)duty);
    }
}

void Charger::setPwm(int duty) {
    _currentPwmDuty = constrain(duty, 0, MAX_PWM_DUTY);
    ledcWrite(PWM_CHANNEL, _currentPwmDuty);
}

void Charger::setFan(int duty) {
    _fanDuty = constrain(duty, 0, MAX_FAN_DUTY);
    ledcWrite(FAN_PWM_CHANNEL, _fanDuty);
}

void Charger::setPsu(bool on) {
    bool isActuallyOn = (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON);
    if (on) {
        if (!isActuallyOn) {
            unsigned long now = millis();
            if (now - _lastPsuOffTime >= PSU_RESTART_COOLDOWN_MS) {
                digitalWrite(ATX_PS_ON_PIN, ATX_PSU_ON);
                Serial.println("PSU turned ON");
            }
        }
    } else {
        if (isActuallyOn) {
            digitalWrite(ATX_PS_ON_PIN, ATX_PSU_OFF);
            _lastPsuOffTime = millis();
            Serial.println("PSU turned OFF");
        }
    }
}

bool Charger::isPsuOn() const {
    return (digitalRead(ATX_PS_ON_PIN) == ATX_PSU_ON);
}

unsigned long Charger::chargeTime() const {
    if (_state == IDLE) return 0;
    if (_state == CHARGED_COMPLETE || _state == ERROR_STATE) return _chargeEndTime - _chargeStartTime;
    return millis() - _chargeStartTime;
}

void Charger::handleError(ErrorType_t type, const char* msg) {
    if (_state == ERROR_STATE) return;
    Serial.print("ERROR: "); Serial.println(msg);
    _lastError = type;
    _state = ERROR_STATE;
    setPwm(0);
    setPsu(false);
    setFan(MAX_FAN_DUTY); // Cool down after error
    _chargeEndTime = millis();
    _storage.save(_lifetimeAh + _integratedAh, _lifetimeWh + _integratedWh);
}

int Charger::analogReadAveraged(int pin) {
    long sum = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i) {
        sum += analogRead(pin);
    }
    return (int)(sum / ADC_SAMPLES);
}
