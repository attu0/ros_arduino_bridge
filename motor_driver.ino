/***********************************************************
  Motor driver definitions — RMCS-2305 for IG45 Encoder Motor

  Follows the EXACT same #ifdef structure as attu0/ros_arduino_bridge
  motor_driver.ino so it slots in as a drop-in replacement.

  To use: add  #define RMCS2305_MOTOR_DRIVER
           and #define USE_BASE
  at the top of ROSArduinoBridge.ino

  RMCS-2305 control logic (different from L298N):
    SLEEP HIGH = motor coasting / DISABLED
    SLEEP LOW  = motor ACTIVE
    DIR   HIGH = forward
    DIR   LOW  = reverse
    PWM   0-255 = speed

  FIX for "o 0 120 not working":
    Each motor's SLEEP pin is controlled INDEPENDENTLY.
    Setting motor LEFT speed = 0 puts LEFT to sleep,
    but RIGHT is woken and driven separately.
    They never share or block each other.
***********************************************************/

#ifdef USE_BASE

#ifdef RMCS2305_MOTOR_DRIVER

/* ── Init: both motors start in sleep (disabled) state ── */
void initMotorController() {
  pinMode(LEFT_MOTOR_SLEEP,  OUTPUT);
  pinMode(LEFT_MOTOR_DIR,    OUTPUT);
  pinMode(LEFT_MOTOR_PWM,    OUTPUT);

  pinMode(RIGHT_MOTOR_SLEEP, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR,   OUTPUT);
  pinMode(RIGHT_MOTOR_PWM,   OUTPUT);

  // RMCS-2305: HIGH on SLEEP = coast/disabled
  digitalWrite(LEFT_MOTOR_SLEEP,  HIGH);
  digitalWrite(RIGHT_MOTOR_SLEEP, HIGH);
  analogWrite(LEFT_MOTOR_PWM,  0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
}

/* ── setMotorSpeed: mirrors L298N version exactly ──────────
   i   = LEFT (0) or RIGHT (1)
   spd = -255 (full reverse) to +255 (full forward)
         0 = stop this motor (sleep its channel)
   This is called independently per motor, so
   o 0 120  →  setMotorSpeed(LEFT,0)  then setMotorSpeed(RIGHT,120)
   Each call only touches its own SLEEP/DIR/PWM pins.        */
void setMotorSpeed(int i, int spd) {

  // Clamp
  if (spd > PWM_MAX)  spd =  PWM_MAX;
  if (spd < -PWM_MAX) spd = -PWM_MAX;

  // Choose pins for this motor
  byte sleepPin = (i == LEFT) ? LEFT_MOTOR_SLEEP  : RIGHT_MOTOR_SLEEP;
  byte dirPin   = (i == LEFT) ? LEFT_MOTOR_DIR    : RIGHT_MOTOR_DIR;
  byte pwmPin   = (i == LEFT) ? LEFT_MOTOR_PWM    : RIGHT_MOTOR_PWM;

  if (spd == 0) {
    // Stop THIS motor only — does not affect the other motor
    analogWrite(pwmPin, 0);
    digitalWrite(sleepPin, HIGH);   // put this channel to sleep
    return;
  }

  // Wake THIS channel
  digitalWrite(sleepPin, LOW);

  if (spd > 0) {
    // Forward
    int pwm = (spd < PWM_MIN_MOVE) ? PWM_MIN_MOVE : spd;
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin,  pwm);
  } else {
    // Reverse
    int pwm = (-spd < PWM_MIN_MOVE) ? PWM_MIN_MOVE : -spd;
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin,  pwm);
  }
}

/* ── Convenience wrapper (same as all other drivers) ── */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT,  leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

#elif defined L298_MOTOR_DRIVER

/* ── Original L298N block kept for reference ── */
void initMotorController() {
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
  digitalWrite(LEFT_MOTOR_ENABLE,  HIGH);
}

void setMotorSpeed(int i, int spd) {
  unsigned char reverse = 0;
  if (spd < 0) { spd = -spd; reverse = 1; }
  if (spd > 255) spd = 255;

  if (i == LEFT) {
    if (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD,  spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
    else               { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD,  0); }
  } else {
    if (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD,  spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
    else               { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD,  0); }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT,  leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

#else
  #error A motor driver must be selected!
#endif

#endif  // USE_BASE
