#ifndef IMU_H_
#define IMU_H_

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

void IMUsetup();
void IMUloop();

extern Quaternion q;           // [w, x, y, z]         quaternion container
extern VectorInt16 aa;         // [x, y, z]            accel sensor measurements
extern VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
extern VectorFloat gravity;    // [x, y, z]            gravity vector
extern float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#endif //IMU_H