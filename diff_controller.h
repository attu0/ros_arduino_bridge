/* diff_controller.h
 *
 * PID closed-loop speed controller for differential drive.
 * Adapted for IG45 50W 125:1 encoder motor.
 *
 * Speed unit: encoder counts per PID loop
 *   At LOOP_RATE=30 Hz, 50 RPM = (6500 × 50) / (60 × 30) = 180 counts/loop
 *   Practical starting command: m 108 108  (approx 30 RPM)
 *
 * PID update command: u <Kp> <Kd> <Ki> <Ko>
 *   Note: original uses 'u' not 'p' — matches commands.h UPDATE_PID = 'u'
 */

#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

// ── Loop timing ──────────────────────────────────────────────
#define LOOP_RATE         30              // Hz
#define PID_INTERVAL      (1000 / LOOP_RATE)   // 33 ms per PID tick

// ── Default PID Gains ────────────────────────────────────────
// Tune via serial: u <Kp> <Kd> <Ki> <Ko>
// Start with Kp only. Add Kd if oscillating. Add Ki for steady-state error.
#define Kp_DEFAULT   20
#define Kd_DEFAULT    0
#define Ki_DEFAULT    0
#define Ko_DEFAULT   50

// ── Auto-stop timeout ────────────────────────────────────────
// Motors stop if no command received within this time
#define AUTO_STOP_INTERVAL   2000   // ms

/* PID state — one instance per motor */
typedef struct {
  double TargetTicksPerFrame;   // desired ticks per PID loop (set by 'm')
  long   Encoder;               // current encoder count (snapshot)
  long   PrevEnc;               // previous encoder count
  int    PrevInput;             // previous ticks-per-frame (for derivative)
  int    ITerm;                 // integral accumulator
  long   output;                // final PID output → PWM
} SetPointInfo;

extern SetPointInfo leftPID, rightPID;
extern int Kp, Kd, Ki, Ko;

void resetPID();
void doPID(SetPointInfo* p);
void updatePID();

#endif
