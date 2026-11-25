// Quadrature decoding at 4x resolution.

#include <util/atomic.h>
#include "config.h"
#include "commands.h"
#include "encoder_driver.h"

static volatile long leftCount  = 0;
static volatile long rightCount = 0;

// Step for each (previous AB << 2) | current AB.
static const int8_t ENC_STATES[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

static inline uint8_t readAB(uint8_t port, uint8_t a, uint8_t b)
{
  return ((port >> a) & 1) | (((port >> b) & 1) << 1);
}

ISR(PCINT2_vect)   // left, PORTD
{
  static uint8_t last = 0;
  last = (last << 2) | readAB(PIND, LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
  leftCount += ENC_STATES[last & 0x0f];
}

ISR(PCINT1_vect)   // right, PORTC
{
  static uint8_t last = 0;
  last = (last << 2) | readAB(PINC, RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
  rightCount += ENC_STATES[last & 0x0f];
}

void initEncoders()
{
  DDRD  &= ~(_BV(LEFT_ENC_PIN_A) | _BV(LEFT_ENC_PIN_B));
  PORTD |=   _BV(LEFT_ENC_PIN_A) | _BV(LEFT_ENC_PIN_B);
  DDRC  &= ~(_BV(RIGHT_ENC_PIN_A) | _BV(RIGHT_ENC_PIN_B));
  PORTC |=   _BV(RIGHT_ENC_PIN_A) | _BV(RIGHT_ENC_PIN_B);

  PCMSK2 |= _BV(LEFT_ENC_PIN_A)  | _BV(LEFT_ENC_PIN_B);
  PCMSK1 |= _BV(RIGHT_ENC_PIN_A) | _BV(RIGHT_ENC_PIN_B);
  PCICR  |= _BV(PCIE1) | _BV(PCIE2);
}

long readEncoder(int i)
{
  long v;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = (i == LEFT) ? leftCount : rightCount; }   // 32-bit read is not atomic on AVR
  return v;
}

void resetEncoder(int i)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (i == LEFT) leftCount = 0;
    else           rightCount = 0;
  }
}

void resetEncoders()
{
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
