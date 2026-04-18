/* ROSArduinoBridge.ino
 *
 * Board  : Arduino Mega 2560
 * Driver : Rhino RMCS-2305 Dual DC Motor Driver (20A, 6V-30V)
 * Motor  : Robokits IG45 50W 50RPM 12V Planetary Encoder Motor (×2)
 *          Encoder: 52 CPR × 125 gear ratio = 6500 counts/output rev
 *
 * Based on: attu0/ros_arduino_bridge (joshnewans fork)
 * Compatible with: serial_motor_demo ROS2 package
 *
 * ── IMPORTANT: FIX FOR "o 0 120 not working" ────────────────
 * Root cause: each motor's SLEEP pin must be controlled
 * INDEPENDENTLY. When motor LEFT = 0, only LEFT goes to sleep.
 * RIGHT is woken and driven normally. See motor_driver.ino.
 * ─────────────────────────────────────────────────────────────
 *
 * SERIAL COMMANDS (57600 baud + CR):
 *   e              → encoder counts: "left right\r"
 *   m <l> <r>      → closed-loop speed (counts/loop)  → "OK\r"
 *   o <l> <r>      → raw PWM -255 to 255              → "OK\r"
 *   r              → reset encoders                   → "OK\r"
 *   u <Kp><Kd><Ki><Ko> → update PID                  → "OK\r"
 *   b              → baud rate                        → "57600\r"
 *   p              → ping                             → "0\r"
 *   a <pin>        → analog read                      → value\r
 *   d <pin>        → digital read                     → value\r
 *   w <pin> <val>  → digital write                    → "OK\r"
 *   x <pin> <val>  → analog write (PWM)               → "OK\r"
 *   c <pin> <mode> → pin mode (0=input,1=output)      → "OK\r"
 *
 * ROS2 launch:
 *   ros2 run serial_motor_demo driver --ros-args \
 *     -p serial_port:=/dev/ttyACM0 \
 *     -p baud_rate:=57600 \
 *     -p encoder_cpr:=6500 \
 *     -p loop_rate:=30
 *
 * WIRING:
 *   RMCS-2305  →  Mega   |  IG45 M1 Encoder  →  Mega
 *   SLEEP1     →  22     |  ENC_A1            →  2 (INT4)
 *   DIR1       →  24     |  ENC_B1            →  4
 *   PWM1       →   6     |  5V                →  5V
 *   PWM2       →   7     |  GND               →  GND
 *   DIR2       →  25     |
 *   SLEEP2     →  23     |  IG45 M2 Encoder  →  Mega
 *   GND        →  GND    |  ENC_A2            →  3 (INT5)
 *   5V(VCC)    →  5V     |  ENC_B2            →  5
 *                         |  5V                →  5V
 *   12V PSU    →  RMCS-2305 VCC+GND power terminals
 *   Motor 1    →  M1A, M1B
 *   Motor 2    →  M2A, M2B
 */

/* ── Driver selection — MUST be defined before includes ── */
#define USE_BASE
#define RMCS2305_MOTOR_DRIVER

/* ── Includes ─────────────────────────────────────────────── */
#include "commands.h"
#include "diff_controller.h"
#include "encoder_driver.h"
#include "motor_driver.h"

/* ── PID state ───────────────────────────────────────────── */
SetPointInfo leftPID, rightPID;
int Kp = Kp_DEFAULT;
int Kd = Kd_DEFAULT;
int Ki = Ki_DEFAULT;
int Ko = Ko_DEFAULT;

/* ── Timing ──────────────────────────────────────────────── */
unsigned long lastMotorCommand = AUTO_STOP_INTERVAL;
unsigned long lastPIDTime      = 0;
boolean moving                 = false;

/* ── Serial input ────────────────────────────────────────── */
#define BUFFER_SIZE 64
char    inputBuffer[BUFFER_SIZE];
int     inputIndex   = 0;
boolean inputComplete = false;

/* ═══════════════════════════════════════════════════════════
   PID IMPLEMENTATION
   ═══════════════════════════════════════════════════════════ */
void resetPID() {
  leftPID.TargetTicksPerFrame  = 0;
  rightPID.TargetTicksPerFrame = 0;
  leftPID.Encoder   = readEncoder(LEFT);
  rightPID.Encoder  = readEncoder(RIGHT);
  leftPID.PrevEnc   = leftPID.Encoder;
  rightPID.PrevEnc  = rightPID.Encoder;
  leftPID.ITerm     = 0;
  rightPID.ITerm    = 0;
  leftPID.output    = 0;
  rightPID.output   = 0;
  leftPID.PrevInput = 0;
  rightPID.PrevInput = 0;
}

void doPID(SetPointInfo* p) {
  long  output;
  int   ticksThisFrame = p->Encoder - p->PrevEnc;
  p->PrevEnc = p->Encoder;

  int Perror = p->TargetTicksPerFrame - ticksThisFrame;

  // PID formula (same as original hbrobotics/ros_arduino_bridge)
  output = (Kp * Perror
            - Kd * (ticksThisFrame - p->PrevInput)
            + p->ITerm) / Ko;

  p->PrevInput = ticksThisFrame;

  // Clamp and windup guard
  if      (output >=  PWM_MAX) { output =  PWM_MAX; }
  else if (output <= -PWM_MAX) { output = -PWM_MAX; }
  else                          { p->ITerm += Ki * Perror; }

  p->output = output;
}

void updatePID() {
  // Snapshot encoders (thread-safe)
  leftPID.Encoder  = readEncoder(LEFT);
  rightPID.Encoder = readEncoder(RIGHT);

  if (!moving) {
    if (leftPID.PrevEnc  != leftPID.Encoder)  doPID(&leftPID);
    if (rightPID.PrevEnc != rightPID.Encoder) doPID(&rightPID);
    return;
  }

  doPID(&leftPID);
  doPID(&rightPID);
  setMotorSpeeds(leftPID.output, rightPID.output);
}

/* ═══════════════════════════════════════════════════════════
   SERIAL COMMAND RUNNER
   ═══════════════════════════════════════════════════════════ */
void runCommand() {
  int    args[4] = {0, 0, 0, 0};
  int    nArgs   = 0;
  char*  token;
  char   cmd     = inputBuffer[0];

  // Parse space-separated integer arguments after command char
  token = strtok(inputBuffer + 2, " ");  // skip cmd + space
  while (token != NULL && nArgs < 4) {
    args[nArgs++] = atoi(token);
    token = strtok(NULL, " ");
  }

  switch (cmd) {

    /* ── e: Read encoder counts ─────────────────────────── */
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;

    /* ── r: Reset encoders ──────────────────────────────── */
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;

    /* ── o: Raw PWM (open-loop) — FIX applied here ──────
     *   o 120 0   → LEFT runs,  RIGHT stopped independently
     *   o 0 120   → LEFT stopped, RIGHT runs independently
     *   o 120 120 → both run
     *   Each motor's SLEEP controlled separately in setMotorSpeed()
     * ──────────────────────────────────────────────────── */
    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      resetPID();
      moving = false;   // raw mode bypasses PID
      // Call setMotorSpeeds → calls setMotorSpeed per motor independently
      setMotorSpeeds(args[0], args[1]);
      Serial.println("OK");
      break;

    /* ── m: Closed-loop speed (counts per PID loop) ─────── */
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (args[0] == 0 && args[1] == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = false;
      } else {
        moving = true;
      }
      leftPID.TargetTicksPerFrame  = args[0];
      rightPID.TargetTicksPerFrame = args[1];
      Serial.println("OK");
      break;

    /* ── u: Update PID gains (note: 'u' not 'p' per commands.h) */
    case UPDATE_PID:
      if (nArgs >= 4) {
        Kp = args[0];
        Kd = args[1];
        Ki = args[2];
        Ko = args[3];
      }
      Serial.println("OK");
      break;

    /* ── b: Get baud rate ───────────────────────────────── */
    case GET_BAUDRATE:
      Serial.println(57600);
      break;

    /* ── p: Ping ────────────────────────────────────────── */
    case PING:
      Serial.println("0");
      break;

    /* ── a: Analog read ─────────────────────────────────── */
    case ANALOG_READ:
      Serial.println(analogRead(args[0]));
      break;

    /* ── d: Digital read ────────────────────────────────── */
    case DIGITAL_READ:
      Serial.println(digitalRead(args[0]));
      break;

    /* ── w: Digital write ───────────────────────────────── */
    case DIGITAL_WRITE:
      if (args[1] == 0)      digitalWrite(args[0], LOW);
      else                   digitalWrite(args[0], HIGH);
      Serial.println("OK");
      break;

    /* ── x: Analog write (PWM) ──────────────────────────── */
    case ANALOG_WRITE:
      analogWrite(args[0], args[1]);
      Serial.println("OK");
      break;

    /* ── c: Pin mode ────────────────────────────────────── */
    case PIN_MODE:
      if (args[1] == 0)      pinMode(args[0], INPUT);
      else                   pinMode(args[0], OUTPUT);
      Serial.println("OK");
      break;

    default:
      Serial.println("Invalid command");
      break;
  }
}

/* ═══════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(57600);      // Must match serial_motor_demo baud_rate

  initMotorController();    // RMCS-2305: sets all pins, both channels to SLEEP
  initEncoders();           // IG45: attach ISR on INT4 (pin2), INT5 (pin3)
  resetPID();

  Serial.println("ROSArduinoBridge READY");
  Serial.println("Board : Arduino Mega 2560");
  Serial.println("Driver: RMCS-2305  Motor: IG45 125:1  CPR: 6500");
  Serial.println("Commands: e r o m u b p a d w x c");
}

/* ═══════════════════════════════════════════════════════════
   LOOP
   ═══════════════════════════════════════════════════════════ */
void loop() {

  /* ── 1. Accumulate serial bytes until CR/LF ─────────── */
  while (Serial.available() > 0 && !inputComplete) {
    char c = Serial.read();

    if (c == '\r' || c == '\n') {
      if (inputIndex > 0) {
        inputBuffer[inputIndex] = '\0';
        inputComplete = true;
      }
    } else {
      if (inputIndex < BUFFER_SIZE - 1) {
        inputBuffer[inputIndex++] = c;
      }
    }
  }

  /* ── 2. Execute command when line complete ───────────── */
  if (inputComplete) {
    runCommand();
    memset(inputBuffer, 0, BUFFER_SIZE);
    inputIndex    = 0;
    inputComplete = false;
  }

  /* ── 3. PID update at 30 Hz ─────────────────────────── */
  if (millis() > lastPIDTime + PID_INTERVAL) {
    lastPIDTime = millis();
    updatePID();
  }

  /* ── 4. Auto-stop if no command in 2 seconds ─────────── */
  if (millis() > lastMotorCommand + AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = false;
  }
}
