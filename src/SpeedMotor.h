#ifndef SPEEDMOTOR_H
#define SPEEDMOTOR_H



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