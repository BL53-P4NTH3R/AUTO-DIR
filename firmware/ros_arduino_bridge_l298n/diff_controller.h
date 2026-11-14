// Per-wheel speed PID in rad/s, run at PID_RATE.

#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

static const float COUNTS_TO_RAD = 2.0f * PI / ENC_COUNTS_PER_REV;

typedef struct {
  float kp, ki, kd;
  int   minPwm;
} PidGains;

typedef struct {
  float target;     // rad/s
  long  encoder;
  long  prevEnc;
  float speed;      // rad/s, measured last frame
  float prevSpeed;
  float error;      // rad/s
  float iTerm;      // PWM
  int   output;     // PWM
} SetPointInfo;

PidGains leftGains  = {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN_PWM};
PidGains rightGains = {DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_MIN_PWM};

SetPointInfo leftPID, rightPID;

unsigned char moving = 0;

void resetWheel(SetPointInfo *p, int side)
{
  p->target    = 0;
  p->encoder   = readEncoder(side);
  p->prevEnc   = p->encoder;
  p->speed     = 0;
  p->prevSpeed = 0;
  p->error     = 0;
  p->iTerm     = 0;
  p->output    = 0;
}

void resetPID()
{
  resetWheel(&leftPID, LEFT);
  resetWheel(&rightPID, RIGHT);
}

void doPID(SetPointInfo *p, const PidGains *g)
{
  p->error = p->target - p->speed;

  const float ff   = (p->target > 0) ? g->minPwm : (p->target < 0) ? -g->minPwm : 0;
  const float base = ff + g->kp * p->error - g->kd * (p->speed - p->prevSpeed) * PID_RATE;
  const float i    = constrain(p->iTerm + g->ki * p->error / PID_RATE, -MAX_PWM, MAX_PWM);
  float u = base + i;

  // Anti-windup: hold the integral while saturated.
  if ((u > MAX_PWM && p->error > 0) || (u < -MAX_PWM && p->error < 0))
    u = base + p->iTerm;
  else
    p->iTerm = i;

  p->output = (int)constrain(u, -MAX_PWM, MAX_PWM);
}

void measureWheel(SetPointInfo *p, int side)
{
  p->encoder   = readEncoder(side);
  p->prevSpeed = p->speed;
  p->speed     = (p->encoder - p->prevEnc) * COUNTS_TO_RAD * PID_RATE;
  p->prevEnc   = p->encoder;
}

void updatePID()
{
  measureWheel(&leftPID, LEFT);
  measureWheel(&rightPID, RIGHT);

  if (!moving) return;   // stopped or raw PWM: measure only

  doPID(&leftPID, &leftGains);
  doPID(&rightPID, &rightGains);
  setMotorSpeeds(leftPID.output, rightPID.output);
}

#endif
