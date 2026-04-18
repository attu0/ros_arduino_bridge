#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

// ── Encoder Pins ─────────────────────────────────────────────
#define ENC_A1   2      // LEFT  motor — interrupt pin (INT4)
#define ENC_B1   4      // LEFT  motor — direction read
#define ENC_A2   3      // RIGHT motor — interrupt pin (INT5)
#define ENC_B2   5      // RIGHT motor — direction read

// ── IG45 Encoder Specs ────────────────────────────────────────
#define ENCODER_CPR      52.0     // Counts Per Rev, base motor shaft (13PPR × 4)
#define GEAR_RATIO      125.0     // Planetary gearbox ratio (50 RPM model)
#define COUNTS_PER_REV  (ENCODER_CPR * GEAR_RATIO)   // = 6500 per output shaft rev

// ── Function Declarations ─────────────────────────────────────
void initEncoders();
long readEncoder(int i);
void resetEncoders();

#endif
