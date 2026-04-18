#include "encoder_driver.h"

// ── Volatile encoder counters (modified inside ISR) ──────────
volatile long encoderCount1 = 0;   // LEFT  motor
volatile long encoderCount2 = 0;   // RIGHT motor

// ── ISR: LEFT motor (ENC_A1 = pin 2 = INT4) ─────────────────
void encoder_ISR1() {
  if (digitalRead(ENC_B1)) encoderCount1++;
  else                      encoderCount1--;
}

// ── ISR: RIGHT motor (ENC_A2 = pin 3 = INT5) ────────────────
void encoder_ISR2() {
  if (digitalRead(ENC_B2)) encoderCount2++;
  else                      encoderCount2--;
}

// ── Init: configure pins and attach interrupts ────────────────
void initEncoders() {
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  // RISING edge on channel A → read channel B for direction
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoder_ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoder_ISR2, RISING);
}

// ── Read encoder (interrupt-safe) ────────────────────────────
long readEncoder(int i) {
  long val;
  noInterrupts();
  val = (i == LEFT) ? encoderCount1 : encoderCount2;
  interrupts();
  return val;
}

// ── Reset both encoders to zero ───────────────────────────────
void resetEncoders() {
  noInterrupts();
  encoderCount1 = 0;
  encoderCount2 = 0;
  interrupts();
}
