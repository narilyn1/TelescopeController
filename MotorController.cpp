

#include "MotorController.h"
#include <Arduino.h>

const static unsigned long s_updateMicros[4] = {HALFSTEP_RATE_USEC / 32, HALFSTEP_RATE_USEC / 10, HALFSTEP_RATE_USEC, HALFSTEP_RATE_USEC * 2};
const double s_degreePerStep = 7.5 / 120.0 / 144.0 / 2.0; // HalfStep
#define GOTO_TARGET_THRESHOLD_DEG (0.00278)  // 10" = 1 / 3600
#define MOTOR_SLEEP_USEC (10000)

MotorController::MotorController(unsigned short pin0, unsigned short pin1, unsigned short pin2, unsigned short pin3)
: m_motorStep(0), m_lastMotorStep(-1), m_direction(0.0d), m_targetDirection(0.0d), m_motorSpeed(MOTOR_SPEED_FASTEST), m_motorState(MOTOR_STATE_STOP),
  m_directionInversion(false), m_isInGoto(false), m_isInAdjustRotation(0), m_adjustLastUpdate(0), m_adjustUntilMsec(0), m_backlashStep(0) {
  m_pin[0] = pin0;
  m_pin[1] = pin1;
  m_pin[2] = pin2;
  m_pin[3] = pin3;
}

MotorController::~MotorController() {
  
}

void MotorController::init() {
  pinMode(m_pin[0], OUTPUT);
  pinMode(m_pin[1], OUTPUT);
  pinMode(m_pin[2], OUTPUT);
  pinMode(m_pin[3], OUTPUT);

  m_lastUpdate = micros();
  sync();
}

void MotorController::update() {
  unsigned long nextUpdateTime = m_lastUpdate + (isInBacklash() == false ? s_updateMicros[m_motorSpeed] : s_updateMicros[MOTOR_SPEED_FASTEST]);
  unsigned long currentTime = micros();

  if(currentTime > nextUpdateTime) {
    if(m_motorState == MOTOR_STATE_ROTATE_CW) {
      cwStep();
      m_direction -= s_degreePerStep;
    } else if(m_motorState == MOTOR_STATE_ROTATE_CCW) {
      ccwStep();
      m_direction += s_degreePerStep;
    }
    if(m_isInGoto) {
      if(fabs(getTargetDirection() - getDirection()) < GOTO_TARGET_THRESHOLD_DEG) {
        // Reached to target
        m_isInGoto = false;
        rotateStop();
      }
    }
    sync();
    m_lastUpdate = nextUpdateTime;
  }

  // process adjust rotation backlush
  if(0 != m_isInAdjustRotation) {
    unsigned long nextUpdateTime = m_adjustLastUpdate + s_updateMicros[MOTOR_SPEED_SLOWEST];
    if(currentTime > nextUpdateTime) {
      if(m_isInAdjustRotation > 0) {
        cwStep();
      } else {
        ccwStep();
      }
      m_adjustLastUpdate = nextUpdateTime;
    }
    if(millis() > m_adjustUntilMsec) { // finish adjustment
      m_isInAdjustRotation = 0;
      m_adjustLastUpdate = 0;
      m_adjustUntilMsec = 0;
    }
  }

  // process motor sleep
  if(m_lastSync != 0 && (micros() > m_lastSync + MOTOR_SLEEP_USEC)) {
    sleepMotor();
  }
}

void MotorController::goTo() {
  double diff = getTargetDirection() - getDirection();
  while(diff > 180.0) {
    diff -= 360.0;
  }
  while(diff < -180.0) {
    diff += 360.0;
  }

  if(fabs(diff) < GOTO_TARGET_THRESHOLD_DEG) {
    // Already reached, do nothing
  } else if(diff > 0) {
    ccwRotate();
    m_isInGoto = true;
    m_motorSpeed = MOTOR_SPEED_FASTEST;
  } else {
    cwRotate();
    m_isInGoto = true;
    m_motorSpeed = MOTOR_SPEED_FASTEST;
  }
}

void MotorController::sync() {
  // 先にLOWにする
  if(m_lastMotorStep != m_motorStep) {
    switch(m_motorStep) {
      case 0:
        digitalWrite(m_pin[1], LOW);
        digitalWrite(m_pin[2], LOW);
        digitalWrite(m_pin[3], LOW);
        digitalWrite(m_pin[0], HIGH);
        break;
      case 1:
        digitalWrite(m_pin[2], LOW);
        digitalWrite(m_pin[3], LOW);
        digitalWrite(m_pin[0], HIGH);
        digitalWrite(m_pin[1], HIGH);
        break;
      case 2:
        digitalWrite(m_pin[0], LOW);
        digitalWrite(m_pin[2], LOW);
        digitalWrite(m_pin[3], LOW);
        digitalWrite(m_pin[1], HIGH);
        break;
      case 3:
        digitalWrite(m_pin[0], LOW);
        digitalWrite(m_pin[3], LOW);
        digitalWrite(m_pin[1], HIGH);
        digitalWrite(m_pin[2], HIGH);
        break;
      case 4:
        digitalWrite(m_pin[0], LOW);
        digitalWrite(m_pin[1], LOW);
        digitalWrite(m_pin[3], LOW);
        digitalWrite(m_pin[2], HIGH);
        break;
      case 5:
        digitalWrite(m_pin[0], LOW);
        digitalWrite(m_pin[1], LOW);
        digitalWrite(m_pin[2], HIGH);
        digitalWrite(m_pin[3], HIGH);
        break;
      case 6:
        digitalWrite(m_pin[0], LOW);
        digitalWrite(m_pin[1], LOW);
        digitalWrite(m_pin[2], LOW);
        digitalWrite(m_pin[3], HIGH);
        break;
      case 7:
        digitalWrite(m_pin[1], LOW);
        digitalWrite(m_pin[2], LOW);
        digitalWrite(m_pin[0], HIGH);
        digitalWrite(m_pin[3], HIGH);
        break;
    }
    m_lastMotorStep = m_motorStep;
    m_lastSync = micros();
  }
}

void MotorController::sleepMotor()
{
  digitalWrite(m_pin[1], LOW);
  digitalWrite(m_pin[2], LOW);
  digitalWrite(m_pin[0], LOW);
  digitalWrite(m_pin[3], LOW);
  m_lastSync = 0;
}

void MotorController::cwAdjust(int msec)
{
  m_isInAdjustRotation = 1;
  m_adjustLastUpdate = micros();
  m_adjustUntilMsec = millis() + msec;
}

void MotorController::ccwAdjust(int msec)
{
  m_isInAdjustRotation = -1;
  m_adjustLastUpdate = micros();
  m_adjustUntilMsec = millis() + msec;
}
