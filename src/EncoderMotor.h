#ifndef ENCODERMOTOR_H
#define ENCODERMOTOR_H

#include <Arduino.h>

class EncoderMotor {
public:
    EncoderMotor(byte encA_pin, byte encB_pin);

    void updatePosition(byte encA_read, byte encB_read);

    byte encA;
    byte encB;

    int position;

    byte encAFlag = 0;
    byte encBFlag = 0;
};

#endif //ENCODERMOTOR_H