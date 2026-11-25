// Serial bridge between ROS 2 (ubot_control) and an Arduino Nano driving one L298N.

#include "config.h"
#include "commands.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "diff_controller.h"

static const unsigned long PID_INTERVAL_US = 1000000UL / PID_RATE;

static unsigned long lastPidUs        = 0;
static unsigned long lastMotorCommand = 0;
static bool          motorsActive     = false;
static bool          fromTerminal     = false;   // last line ended in LF (ROS sends CR only)
static bool          fromV            = false;   // last speed command was 'v' (ROS only sends 'm')

#define CMD_BUF_LEN 32

void stopAll()
{
  stopMotors();
  resetPID();
  moving = 0;
  motorsActive = false;
}

// Parse "a:b:c:d" into v[]; empty fields keep their value.
void parseFields(char *s, float v[4])
{
  for (int i = 0; i < 4 && s; i++) {
    char *next = strchr(s, ':');
    if (next) *next++ = '\0';
    if (*s) v[i] = atof(s);
    s = next;
  }
}

void setGains(PidGains *g, char *s)   // Kp:Ki:Kd:MinPwm
{
  float v[4] = {g->kp, g->ki, g->kd, (float)g->minPwm};
  parseFields(s, v);
  g->kp = v[0]; g->ki = v[1]; g->kd = v[2]; g->minPwm = (int)v[3];
}

void printDiagnostics()
{
  const float l = leftPID.speed, r = rightPID.speed;
  const float floats[] = {leftPID.target, rightPID.target, l, r,
                          leftPID.error, rightPID.error,
                          leftPID.iTerm, rightPID.iTerm};

  Serial.print(F("D "));
  Serial.print(readEncoder(LEFT));  Serial.print(' ');
  Serial.print(readEncoder(RIGHT)); Serial.print(' ');
  for (float f : floats) { Serial.print(f, 2); Serial.print(' '); }
  Serial.print(leftPID.output);  Serial.print(' ');
  Serial.print(rightPID.output); Serial.print(' ');
  Serial.print((l + r) * 0.5f * WHEEL_RADIUS_M, 3); Serial.print(' ');
  Serial.println((r - l) * WHEEL_RADIUS_M / WHEEL_SEPARATION_M, 3);
}

void printMonitor(float lAct, float rAct)   // m/s, actual averaged over the print interval
{
  Serial.print(F("TGT L:"));    Serial.print(leftPID.target * WHEEL_RADIUS_M, 3);
  Serial.print(F(" R:"));       Serial.print(rightPID.target * WHEEL_RADIUS_M, 3);
  Serial.print(F(" | ACT L:")); Serial.print(lAct * WHEEL_RADIUS_M, 3);
  Serial.print(F(" R:"));       Serial.print(rAct * WHEEL_RADIUS_M, 3);
  Serial.print(F(" | PWM L:")); Serial.print(leftPID.output);
  Serial.print(F(" R:"));       Serial.println(rightPID.output);
}

void setTargets(float l, float r)   // rad/s
{
  lastMotorCommand = millis();
  if (l == 0 && r == 0) {
    stopAll();
    return;
  }
  // Clear the integral only on start or reversal, not on every command.
  if (!moving) {
    resetPID();
  } else {
    if (l * leftPID.target  < 0) leftPID.iTerm  = 0;
    if (r * rightPID.target < 0) rightPID.iTerm = 0;
  }
  moving = 1;
  motorsActive = true;
  leftPID.target  = l;
  rightPID.target = r;
}

void runCommand(char *line)
{
  const char cmd = line[0];
  char *save;
  char *a1 = strtok_r(line + 1, " ", &save);
  char *a2 = a1 ? strtok_r(NULL, " ", &save) : NULL;
  if (!a1) a1 = (char *)"";
  if (!a2) a2 = (char *)"";

  switch (cmd) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;

    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(' ');
      Serial.println(readEncoder(RIGHT));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      stopAll();
      Serial.println(F("OK"));
      break;

    case MOTOR_SPEEDS:      // rad/s
      fromV = false;
      setTargets(atof(a1), atof(a2));
      Serial.println(F("OK"));
      break;

    case MOTOR_SPEEDS_MS:   // m/s
      fromV = true;
      setTargets(atof(a1) / WHEEL_RADIUS_M, atof(a2) / WHEEL_RADIUS_M);
      Serial.println(F("OK"));
      break;

    case MOTOR_RAW_PWM:
      lastMotorCommand = millis();
      resetPID();
      moving = 0;
      leftPID.output  = constrain(atol(a1), -MAX_PWM, MAX_PWM);
      rightPID.output = constrain(atol(a2), -MAX_PWM, MAX_PWM);
      motorsActive = (leftPID.output != 0 || rightPID.output != 0);
      setMotorSpeeds(leftPID.output, rightPID.output);
      Serial.println(F("OK"));
      break;

    case UPDATE_PID: {   // Kp:Kd:Ki:MinPwm
      float v[4] = {leftGains.kp, leftGains.kd, leftGains.ki, (float)leftGains.minPwm};
      parseFields(a1, v);
      leftGains.kp = v[0]; leftGains.kd = v[1]; leftGains.ki = v[2]; leftGains.minPwm = (int)v[3];
      rightGains = leftGains;
      Serial.println(F("OK"));
      break;
    }

    case UPDATE_LEFT_PID:
      setGains(&leftGains, a1);
      Serial.println(F("OK"));
      break;

    case UPDATE_RIGHT_PID:
      setGains(&rightGains, a1);
      Serial.println(F("OK"));
      break;

    case READ_DIAGNOSTICS:
      printDiagnostics();
      break;

    default:
      Serial.println(F("Invalid Command"));
      break;
  }
}

void pollSerial()   // run a line on CR or LF; reject over-long lines
{
  static char    line[CMD_BUF_LEN];
  static uint8_t len = 0;
  static bool    overflow = false;

  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      fromTerminal = (c == '\n');
      if (overflow)     Serial.println(F("Command too long"));
      else if (len > 0) { line[len] = '\0'; runCommand(line); }
      len = 0;
      overflow = false;
    } else if (len < CMD_BUF_LEN - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
}

void setup()
{
  initMotorController();
  Serial.begin(BAUDRATE);
  initEncoders();
  resetPID();
  lastPidUs = micros();
}

void loop()
{
  pollSerial();

  const unsigned long now = micros();
  if (now - lastPidUs >= PID_INTERVAL_US) {
    lastPidUs += PID_INTERVAL_US;
    if (now - lastPidUs >= PID_INTERVAL_US) lastPidUs = now;   // resync if a frame was missed
    updatePID();

    static float   monL = 0, monR = 0;
    static uint8_t monN = 0;
    if ((fromTerminal || fromV) && moving) {
      monL += leftPID.speed;
      monR += rightPID.speed;
      if (++monN >= PID_RATE / MONITOR_HZ) {
        printMonitor(monL / monN, monR / monN);
        monL = monR = 0;
        monN = 0;
      }
    } else {
      monL = monR = 0;
      monN = 0;
    }
  }

  if (motorsActive && millis() - lastMotorCommand > AUTO_STOP_MS) stopAll();
}
