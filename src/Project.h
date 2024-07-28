#ifndef PROJECT_H
#define PROJECT_H

#include <Arduino.h>
#include "ODriveArduino.h"

#define LED_PIN 2

//#define PROP_TUNING
//#define INT_TUNING
//#define DERIV_TUNING
//#define POS_PROP_TUNING
//#define THROTTLE_TUNING
//#define STEER_TUNING
//#define ENABLE_MOTOR_CALIBRATION
#define STEER_PROP_TUNING
//#define STEER_INT_TUNING

// ODrive params
#define ODRIVE_UART_TX 26
#define ODRIVE_UART_RX 27
#define ODRIVE_MAX_VELOCITY 7.6f
#define ODRIVE_MAX_TORQUE 12.92f

// Max values for tuning purposes
#define ANGLE_MAX 4
#define KP_MAX 300
#define KI_MAX 1
#define KD_MAX 10
#define POS_KP_MAX 1
#define THROTTLE_GAIN_MAX 0.02
#define STEER_KP_MAX 100
#define STEER_KI_MAX 0.5

// Other constants
#define RAMP_TIME 3e6
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.345
#define MIN_POWER_1 0.32f
#define MIN_POWER_2 0.24f
#define MAX_POWER 1024.0f

// RTOS definitions
#define INCLUDE_eTaskGetState 1

// Motor data structure
struct encoderMotor {
    bool axis;
    float power = 0;
    long position = 0;
    float speed = 0;
    encoderMotor(bool axis_num) {
        axis = axis_num;
    }
};

// Template for stream
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial odrive_serial(Serial1);
ODriveArduino odrive(odrive_serial);

// Globals
bool blinkState = false;
int controlOutput;
bool killSwitchReleased = 1;
unsigned long rampTime = 0;
bool rampEnabled = 0;
float rampLevel = 0;
float m1output;
float m2output;
bool running = 1;
unsigned long elapsedTime = 0;
unsigned long prevTime = 0;
unsigned short loopCycles = 0;
bool motor_calib_state = 0;

// PID Balance values
float targetAngle = 0;
float error = 0;
float prevError = 0;
float angle = 0;
float prevAngle;
float proportional = 0;
float integral = 0;
float derivative = 0;
float Kp = 38; 
float Ki = 0.16;
float Kd = 6;

// PI steering values
float speedDiff = 0;
float steeringError = 0;
float steerKp = 0;
float steerKi = 0;
float steerOutput = 0;
float steerProportional = 0;
float steerIntegral = 0;

// RC control vars
float throttle = 0;
float steering = 0;
float prevThrottle = 0;
float throttleGain = 0.006;
float steeringGain = 0.5;
float throttleOutput = 0;
float calibrationOffset = 0;

// Create encoder motor objects
encoderMotor m1(0);
encoderMotor m2(1);

// Global Functions
template <typename T>
T inRange(T min_val, T var, T max_val) {
  if (var < min_val) {
    return min_val;
  } else if (var > max_val) {
    return max_val;
  } else return var;
}

template <typename T>
bool signToBool(T var) {
  if (var < 0) return 1;
  else return 0;
}

#endif //PROJECT_H