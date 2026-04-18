
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

// ── Select this driver in ROSArduinoBridge.ino ──────────────
// #define RMCS2305_MOTOR_DRIVER

// ── RMCS-2305 Pin Definitions ───────────────────────────────
// Motor LEFT (M1)
#define LEFT_MOTOR_SLEEP   22
#define LEFT_MOTOR_DIR     24
#define LEFT_MOTOR_PWM      6

// Motor RIGHT (M2)
#define RIGHT_MOTOR_SLEEP  23
#define RIGHT_MOTOR_DIR    25
#define RIGHT_MOTOR_PWM     7

// ── Limits ──────────────────────────────────────────────────
// Below PWM_MIN_MOVE the IG45 planetary gearbox won't actually move
#define PWM_MIN_MOVE   35
#define PWM_MAX       255

#endif
