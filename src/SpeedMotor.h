#ifndef SPEEDMOTOR_H
#define SPEEDMOTOR_H

#include <Arduino.h>

#define ZERO_SPEED_TIMEOUT 0.1e6
#define BRAKE_TIME_MULTIPLIER 0.05

extern unsigned long elapsedTime;
extern unsigned long prevTime;
extern unsigned short loopCycles;

class SpeedMotor {
public:
    float getPower();
    bool getDirection();
    long getPosition();
    float getSpeed();
    void resetPosition();
    void setPower(float pwr);
    void setDirection(bool dir);
    void setPosition(long pos);
    void setSpeed(float spd);

private:
    float power = 0;
    bool direction = 0;
    long position = 0;
    float speed = 0;
};

#endif //SPEEDMOTOR_H