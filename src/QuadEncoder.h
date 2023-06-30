#ifndef QUADENCODER_H
#define QUADENCODER_H

#include <Arduino.h>

class QuadEncoder {
public:
    QuadEncoder(byte encA_pin, byte encB_pin);

    void updatePos(byte encA_read, byte encB_read);

    byte encA;
    byte encB;

    byte position;

    byte encAFlag = 0;
    byte encBFlag = 0;
};

#endif //QUADENCODER_H