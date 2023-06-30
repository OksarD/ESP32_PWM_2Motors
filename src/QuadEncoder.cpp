#include "QuadEncoder.h"

// Constructor
QuadEncoder::QuadEncoder(byte encA_pin, byte encB_pin) {
    encA = encA_pin;
    encB = encB_pin;
}

// Update and Read Encoder States
void QuadEncoder::updatePos(byte encA_read, byte encB_read) {
    if (encAFlag == 1) {
        if (encA_read != encB_read) {
            position ++;
        } else {
            position --;
        }
    } else if (encBFlag == 1) {
        if (encA_read != encB_read) {
            position --;
        } else {
            position ++;
        }
    }
}