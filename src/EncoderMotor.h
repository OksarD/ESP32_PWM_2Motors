#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

#include <Arduino.h>

class EncoderMotor {
public:
    void configurePins(byte encA_pin, byte encB_pin, byte enable_pin, byte in1_pin, byte in2_pin);
    void configurePWM(unsigned int pwmFreq_min, unsigned int pwmFreq_max, float power_threshold, byte pwm_channel, byte pwm_resolution);

    void driveMotor(unsigned int pwr, byte dir);
    void driveMotor();
    void stopMotor();
    void updatePosition();

    void setPower(unsigned int pwr);
    void setDirection(byte dir);
    unsigned int getMaxPower();
    
    byte encA;
    byte encB;
    byte enable;
    byte in1;
    byte in2;

    byte channel;
    byte resolution;
    unsigned int freqSteps = 5;
    
    unsigned int power = 0;
    byte direction = 0;
    unsigned int frequency;
    int position = 0;
    
    byte encAFlag = 0;
    byte encBFlag = 0;
private:
    void updateFreq();
    unsigned int stepFunction(unsigned int x);

    unsigned int maxPower;
    unsigned int prevPower = 0;
    unsigned int stepHeight;
    unsigned int pwmFreqMin;
    unsigned int pwmFreqMax;
    unsigned int threshold;
    unsigned int prevFrequency;
    unsigned int powerBuffer;
    unsigned int stepWidth;
};

#endif //ENCODERMOTOR_H