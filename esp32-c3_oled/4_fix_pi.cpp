// --- PI Controller parameters ---
float KP_CC = 200.0f;     // proportional gain for current loop
float KI_CC = 50.0f;      // integral gain for current loop
float KP_CV = 100.0f;     // proportional gain for voltage loop
float KI_CV = 20.0f;      // integral gain for voltage loop

// Integrators
float integralCC = 0.0f;
float integralCV = 0.0f;

// Helper: PI with anti-windup
int piController(float error, float &integral, float kp, float ki) {
  // Integrate
  integral += ki * error;

  // Raw control signal
  float u = kp * error + integral;

  // Clamp with anti-windup
  if (u > MAX_PWM_DUTY) {
    u = MAX_PWM_DUTY;
    if (error > 0) integral -= ki * error; // prevent windup
  } else if (u < 0) {
    u = 0;
    if (error < 0) integral -= ki * error;
  }

  return (int)u;
}

/// replace case charging with :
case CHARGING: {
  // Emergency check
  if (abs(chargingCurrentFiltered) > (desiredCurrentLimit * MAX_ALLOWED_CURRENT_MULTIPLIER + 0.1f)) {
    Serial.println("ERROR: measured current exceeds safe multiplier -> shutting PWM OFF!");
    setPwmDutyCycle(0);
    chargerState = IDLE;
    break;
  }

  int dutyCmd = currentPwmDutyCycle;

  // Decide which loop to use
  if (chargingCurrentFiltered > desiredCurrentLimit) {
    // Current control (CC mode)
    float errI = desiredCurrentLimit - chargingCurrentFiltered;
    dutyCmd = piController(errI, integralCC, KP_CC, KI_CC);
    Serial.print("CC PI -> PWM: "); Serial.println(dutyCmd);
  } else if (batteryVoltageFiltered >= desiredFinalVoltage) {
    // Voltage control (CV mode)
    float errV = desiredFinalVoltage - batteryVoltageFiltered;
    dutyCmd = piController(errV, integralCV, KP_CV, KI_CV);
    Serial.print("CV PI -> PWM: "); Serial.println(dutyCmd);
  } else {
    // Bulk mode: use CC loop until near set voltage
    float errI = desiredCurrentLimit - chargingCurrentFiltered;
    dutyCmd = piController(errI, integralCC, KP_CC, KI_CC);
    Serial.print("Bulk PI -> PWM: "); Serial.println(dutyCmd);
  }

  setPwmDutyCycle(dutyCmd);

  // Periodic paused check
  if (now - lastChargeCheckTime >= CHARGE_CHECK_INTERVAL_MS) {
    Serial.println("Periodic check: pausing PWM to measure unloaded battery voltage...");
    setPwmDutyCycle(0);
    pausedStartTime = now;
    chargerState = PAUSED_CHECK_VOLTAGE;
  }
  break;
}
