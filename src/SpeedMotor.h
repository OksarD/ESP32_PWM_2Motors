#ifndef SPEEDMOTOR_H
#define SPEEDMOTOR_H

#include <Arduino.h>

class SpeedMotor {
public:
    void setupPins(byte sc_pin, byte pwr_pin, byte dir_pin, byte brake_pin);
    void setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution);
    void update();

    byte getSC();
    volatile bool* getInterrupt();
    unsigned short getPower();
    bool getDirection();
    int getPosition();
    float getPCR();
    bool getBrake();
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
    int position = 0;
    unsigned long prevTime = 0;
    unsigned long currentTime = 0;
    bool brake = 0;
    float brakeTimeMultiplier = 0.01;
};

#endif //SPEEDMOTOR_H