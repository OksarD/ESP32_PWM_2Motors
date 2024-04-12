#ifndef SPEEDMOTOR_H
#define SPEEDMOTOR_H

#include <Arduino.h>

class SpeedMotor {
public:
    void setupPins(byte sc_pin, byte pwr_pin, byte dir_pin, byte brake_pin);
    void setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution);
    void driveMotor(unsigned int pwr, bool dir);
    void driveMotor();
    void stopMotor();
    void brakeMotor();
    void update();

    byte getSC();
    volatile bool* getInterrupt();
    unsigned short getPower();
    unsigned short getMaxPower();
    long getPosition();
    int getSpeed();

    void setPower(unsigned int pwr);
    void setDirection(bool dir);

private:
    void boolInvert(bool var);

    byte channel;
    byte resolution;
    unsigned int pwmFreq;
    byte scPin;
    byte dirPin;
    byte brakePin;
    byte enable;
    volatile bool scFlag = 0;
    bool scState = 0;
    unsigned short maxPower;
    unsigned short power = 0;
    bool direction = 0;
    bool targetDirection = 0;
    int position = 0;
};

#endif //SPEEDMOTOR_H