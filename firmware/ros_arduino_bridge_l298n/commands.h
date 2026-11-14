// Serial commands: "<cmd> [arg1] [arg2]\r", one reply line each.

#ifndef COMMANDS_H
#define COMMANDS_H

#define GET_BAUDRATE      'b'   // -> baud rate
#define READ_ENCODERS     'e'   // -> "<left> <right>"
#define RESET_ENCODERS    'r'   // zero encoders and stop -> OK
#define MOTOR_SPEEDS      'm'   // m L R, wheel speed in rad/s -> OK
#define MOTOR_SPEEDS_MS   'v'   // v L R, wheel speed in m/s -> OK
#define MOTOR_RAW_PWM     'o'   // o L R, PWM -255..255 -> OK
#define UPDATE_PID        'u'   // u Kp:Kd:Ki:MinPwm, both wheels -> OK
#define UPDATE_LEFT_PID   'l'   // l Kp:Ki:Kd:MinPwm -> OK
#define UPDATE_RIGHT_PID  'f'   // f Kp:Ki:Kd:MinPwm -> OK
#define READ_DIAGNOSTICS  'q'   // -> "D lEnc rEnc lTgt rTgt lVel rVel lErr rErr lI rI lOut rOut lin ang" (speeds in rad/s)

#define LEFT   0
#define RIGHT  1

#endif
