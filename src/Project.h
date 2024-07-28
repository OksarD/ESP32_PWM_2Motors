#ifndef PROJECT_H
#define PROJECT_H

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
#define RC_BITS 10
#define TARG_OFFSET 0
#define RAMP_TIME 3e6
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.345
#define MIN_POWER_1 0.32f
#define MIN_POWER_2 0.24f
#define MAX_POWER 1024.0f

// Globals
struct encoderMotor {
    float power = 0;
    long position = 0;
    float speed = 0;
};


#endif //PROJECT_H