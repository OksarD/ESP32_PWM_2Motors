#include "Control.h"
#include <Macros.h>

void BalanceLoop() {
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