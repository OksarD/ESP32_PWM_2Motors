#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

#include <Arduino.h>

class EncoderMotor {
public:
    void setupPins(byte encA_pin, byte encB_pin, byte enable_pin, byte in1_pin, byte in2_pin);
    void setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution);
    void driveMotor(unsigned int pwr, bool dir);
    void driveMotor();
    void stopMotor();
    void brakeMotor();
    void updatePosition();

    byte getEncA();
    byte getEncB();
    volatile bool* getInterruptA();
    volatile bool* getInterruptB();
    unsigned short getPower();
    unsigned short getMaxPower();
    bool getDirection();
    long getPosition();

    void setPower(unsigned int pwr);
    void setDirection(bool dir);

private:
    byte channel;
    byte resolution;
    unsigned int pwmFreq;
    byte enable;
    byte dirPin;
    byte brakePin;
    byte in1;
    byte in2;
    volatile bool encAFlag = 0;
    volatile bool encBFlag = 0;

    // Will need to get/set these variables
    byte encA;
    byte encB;
    unsigned short maxPower;
    unsigned short power = 0;
    bool direction = 0;
    long position = 0;
};

#endif //ENCODERMOTOR_H