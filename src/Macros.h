#ifndef MACROS_H
#define MACROS_H

//#include <SoftwareSerial.h>
//#include <HardwareSerial.h>

//#define PROP_TUNING
//#define INT_TUNING
//#define DERIV_TUNING
//#define POS_PROP_TUNING
//#define THROTTLE_TUNING
//#define STEER_TUNING
//#define ENABLE_MOTOR_CALIBRATION

// ODrive params
#define ODRIVE_UART_TX 26
#define ODRIVE_UART_RX 27
#define ODRIVE_MAX_VELOCITY 8.0f
#define ODRIVE_MAX_TORQUE 12.92f

// Max values for tuning purposes
#define ANGLE_MAX 4
#define KP_MAX 300
#define KI_MAX 1
#define KD_MAX 10
#define POS_KP_MAX 1
#define THROTTLE_GAIN_MAX 0.02
#define STEERING_GAIN_MAX 0.5

// Other constants
#define RC_BITS 10
#define TARG_OFFSET 0
#define RAMP_TIME 3e6
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.345
#define MIN_POWER_1 0.32f
#define MIN_POWER_2 0.24f
#define MAX_POWER 1024.0f

#endif //MACROS_H