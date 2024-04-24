#ifndef READI2C_H
#define READI2C_H

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

void IMUinit();
void IMUloop();
void RCinit();
void RCloop();

extern Quaternion q;           // [w, x, y, z]         quaternion container
extern VectorInt16 aa;         // [x, y, z]            accel sensor measurements
extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
extern VectorFloat gravity;    // [x, y, z]            gravity vector
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
extern short rcAnalogs[4];     // [CH1, CH2, CH5, CH6] radio reciever data
extern bool ch3State;
extern bool ch4State;
extern byte ch7State;

#endif //READI2C_H