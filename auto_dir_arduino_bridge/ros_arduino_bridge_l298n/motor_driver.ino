// L298N implementation: PWM on the IN pins, ENA/ENB jumpered high.

#include "config.h"
#include "commands.h"
#include "motor_driver.h"

void initMotorController()
{
  const uint8_t pins[] = {LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD,
                          RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD};
  for (uint8_t p : pins) { digitalWrite(p, LOW); pinMode(p, OUTPUT); }

  TCCR1B |= _BV(WGM12);   // Timer1 fast PWM so pins 9/10 match pins 5/6 (~980 Hz)
}

void setMotorSpeed(int i, int spd)
{
  spd = constrain(spd, -MAX_PWM, MAX_PWM);
  const uint8_t fwd = (i == LEFT) ? LEFT_MOTOR_FORWARD  : RIGHT_MOTOR_FORWARD;
  const uint8_t bwd = (i == LEFT) ? LEFT_MOTOR_BACKWARD : RIGHT_MOTOR_BACKWARD;

  if (spd >= 0) { analogWrite(bwd, 0); analogWrite(fwd, spd);  }
  else          { analogWrite(fwd, 0); analogWrite(bwd, -spd); }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

void stopMotors()
{
  setMotorSpeeds(0, 0);
}
