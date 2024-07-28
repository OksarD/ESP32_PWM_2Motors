#include "Control.h"
#include <Project.h>

void BalanceController() {
  // PID for balancing
  unsigned int deltaTime = elapsedTime - prevTime;

  error = targetAngle - angle;
  proportional = Kp*error;
  integral += Ki*error;
  integral = inRange(-MAX_POWER, integral, MAX_POWER);
  if (loopCycles % 50 == 0) {
    derivative = Kd*1e6*(error - prevError)/deltaTime;
    prevTime = elapsedTime;
    prevError = error;
  } 
  controlOutput = proportional + integral + derivative;
}

void SteeringController() {
  // PI algorithm to control steering
  speedDiff = -m2.speed + m1.speed;
  steeringError = steering - speedDiff;
  steerProportional = steerKp*steeringError;
  steerIntegral += steerKi*steeringError;
  steerIntegral = inRange(-MAX_POWER, steerIntegral, MAX_POWER);
  steerOutput = steerProportional + steerIntegral;
}