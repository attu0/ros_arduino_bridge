/* *************************************************************
   Encoder definitions

   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   ************************************************************ */

#ifdef USE_BASE

#ifdef ROBOGAIA
/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* Create the encoder shield object */
MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode

/* Wrap the encoder reading function */
long readEncoder(int i)
{
  if (i == LEFT)
    return encoders.YAxisGetCount();
  else
    return encoders.XAxisGetCount();
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
  if (i == LEFT)
    return encoders.YAxisReset();
  else
    return encoders.XAxisReset();
}
#elif defined(ARDUINO_ENC_COUNTER)
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; // encoder lookup table

/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR(PCINT2_vect)
{

  static uint8_t enc_last_l = 0;
  static uint8_t enc_last_r = 0;

  // -------- LEFT ENCODER (PD2, PD3) --------
  enc_last_l <<= 2;
  enc_last_l |= (PIND & (3 << 2)) >> 2;
  left_enc_pos += ENC_STATES[(enc_last_l & 0x0f)];

  // -------- RIGHT ENCODER (PD4, PD7) --------
  uint8_t right_state = 0;

  if (PIND & (1 << 4))
    right_state |= 1; // PD4
  if (PIND & (1 << 7))
    right_state |= 2; // PD7

  enc_last_r <<= 2;
  enc_last_r |= right_state;
  right_enc_pos += ENC_STATES[(enc_last_r & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
// ISR (PCINT1_vect){
//       static uint8_t enc_last=0;

// enc_last <<=2; //shift previous state two places
// enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

// 	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
// }

/* Wrap the encoder reading function */
long readEncoder(int i)
{
  if (i == LEFT)
    return left_enc_pos;
  else
    return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i)
{
  if (i == LEFT)
  {
    left_enc_pos = 0L;
    return;
  }
  else
  {
    right_enc_pos = 0L;
    return;
  }
}
#else
#error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders()
{
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
