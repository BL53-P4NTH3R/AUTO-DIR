// User settings: serial, timing, geometry, pins and default PID gains.

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#if !defined(__AVR_ATmega328P__)
  #error "Select Arduino Nano (ATmega328P) as the board."
#endif

#define BAUDRATE 57600UL          // must match baud_rate in the xacro

#define PID_RATE      30          // Hz, PID loop rate
#define AUTO_STOP_MS  2000UL      // stop motors if no 'm'/'o' for this long
#define MONITOR_HZ    5           // target vs actual printout after 'v' (or 'm' with an LF line ending)

#define ENC_COUNTS_PER_REV  1440  // counts per wheel rev ('e' after N turns / N)
#define WHEEL_RADIUS_M      0.0325f   // used by 'v', the monitor printout and 'q'
#define WHEEL_SEPARATION_M  0.20f

// L298N IN pins (ENA/ENB jumpers fitted).
#define LEFT_MOTOR_FORWARD    10  // IN1
#define LEFT_MOTOR_BACKWARD    6  // IN2
#define RIGHT_MOTOR_FORWARD    9  // IN3
#define RIGHT_MOTOR_BACKWARD   5  // IN4

#define MAX_PWM 255

// Encoders: left on PORTD, right on PORTC. Swap A/B if a count runs backwards.
#define LEFT_ENC_PIN_A   PD2      // D2
#define LEFT_ENC_PIN_B   PD3      // D3
#define RIGHT_ENC_PIN_A  PC4      // A4
#define RIGHT_ENC_PIN_B  PC5      // A5

// Kp: PWM per rad/s, Ki: PWM per rad, Kd: PWM per rad/s^2, MinPwm: start-up feedforward.
#define DEFAULT_KP       7.5f
#define DEFAULT_KI       70.0f
#define DEFAULT_KD       0.0f
#define DEFAULT_MIN_PWM  70

#endif
