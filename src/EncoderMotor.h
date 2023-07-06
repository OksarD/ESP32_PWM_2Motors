#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

#include <Arduino.h>

class EncoderMotor {
public:
    void configurePins(byte encA_pin, byte encB_pin, byte enable_pin, byte in1_pin, byte in2_pin);
    void configurePWM(unsigned int pwmFreq_min, unsigned int pwmFreq_max, float power_threshold, byte freq_steps, byte pwm_channel, byte pwm_resolution);

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
    
    unsigned int power = 0;
    byte direction = 0;
    unsigned int frequency;
    int position = 0;
    byte encB_read = 0;
    byte encA_read = 0;
    byte* interruptA = &encAFlag;
    byte* interruptB = &encBFlag;

private:
    void updateFreq();
    unsigned int stepFunction(unsigned int x);

    byte channel;
    byte resolution;
    byte freqSteps;
    unsigned int maxPower;

    unsigned int prevPower = 0;
    unsigned int stepHeight;
    unsigned int pwmFreqMin;
    unsigned int pwmFreqMax;
    unsigned int threshold;
    unsigned int prevFrequency;
    unsigned int powerBuffer;
    int lowBufferEdge;
    int highBufferEdge;

    byte encAFlag = 0;
    byte encBFlag = 0;
};

#endif //ENCODERMOTOR_H