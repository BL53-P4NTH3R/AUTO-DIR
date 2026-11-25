// L298N motor driver, speed -255..255 (positive = forward).

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void stopMotors();

#endif
