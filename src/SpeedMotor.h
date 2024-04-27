#ifndef SPEEDMOTOR_H
#define SPEEDMOTOR_H

#include <Arduino.h>

#define ZERO_SPEED_TIMEOUT 0.1e6
#define BRAKE_TIME_MULTIPLIER 0.01

extern unsigned long elapsedTime;
extern unsigned long prevTime;

class SpeedMotor {
public:
    void setupPins(byte sc_pin, byte pwr_pin, byte dir_pin, byte brake_pin);
    void setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution);
    void update();

    byte getSC();
    volatile bool* getInterrupt();
    unsigned short getPower();
    bool getDirection();
    long getPosition();
    float getPCR();
    bool getBrake();
    float getSpeed();
    void resetPosition();
    void setPower(unsigned int pwr);
    void setDirection(bool dir);

private:

    byte channel;
    byte resolution;
    unsigned int pwmFreq;
    byte scPin;
    byte dirPin;
    byte brakePin;
    byte enable;
    volatile bool scFlag = 0;
    bool scState = 0;
    unsigned short power = 0;
    unsigned short prevPower = 0;
    float powerChangeRate = 0;
    bool direction = 0;
    bool brake = 0;
    long position = 0;
    float speed = 0;
    hw_timer_t* scTimer = timerBegin(0, 80, true);
};

#endif //SPEEDMOTOR_H