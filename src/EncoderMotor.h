#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

#include <Arduino.h>

class EncoderMotor {
public:
    void setupPins(byte encA_pin, byte encB_pin, byte enable_pin, byte in1_pin, byte in2_pin);
    void setupPWM(unsigned int pwmFreq_min, unsigned int pwmFreq_max, byte pwm_channel, byte pwm_resolution);
    void changeFreqParamaters(float max_power_threshold = 0.5, byte frequency_steps = 5);
    void driveMotor(unsigned int pwr, byte dir);
    void driveMotor();
    void stopMotor();
    void updatePosition();

    void setPower(unsigned int pwr);
    void setDirection(byte dir);
    unsigned short int getMaxPower();
    
    byte encA;
    byte encB;
    byte enable;
    byte in1;
    byte in2;
    
    unsigned short int power = 0;
    byte direction = 0;
    unsigned int frequency;
    long position = 0;
    byte* interruptA = &encAFlag;
    byte* interruptB = &encBFlag;

private:
    void updateFreq();
    unsigned short int stepFunction(unsigned int x);

    byte channel;
    byte resolution;
    byte freqSteps;
    unsigned short int maxPower;

    unsigned short int prevPower = 0;
    unsigned short int stepHeight;
    unsigned int pwmFreqMin;
    unsigned int pwmFreqMax;
    unsigned short int threshold;
    unsigned int prevFrequency;
    unsigned short int tolerance;
    int lowToleranceEdge;
    int highToleranceEdge;

    byte encAFlag = 0;
    byte encBFlag = 0;
};

#endif //ENCODERMOTOR_H