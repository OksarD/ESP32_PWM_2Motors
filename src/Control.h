#ifndef CONTROL_H
#define CONTROL_H

void BalanceLoop();
void SteeringLoop();

extern float error;
extern float targetAngle;
extern float angle;
extern float proportional;
extern float integral;
extern float Kp;
extern float Ki;
extern float Kd;
extern unsigned short loopCycles;
extern float derivative;
extern float prevError;
extern unsigned long elapsedTime;
extern unsigned long prevTime;
extern int controlOutput;

extern float inRange(float min_val, float var, float max_val);

#endif // CONTROL_H