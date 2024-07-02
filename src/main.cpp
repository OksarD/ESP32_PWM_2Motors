#include <Arduino.h>
#include <SpeedMotor.h>
#include <ReadI2C.h>
#include "ODriveArduino.h"
//#include <SoftwareSerial.h>
//#include <HardwareSerial.h>

//#define PROP_TUNING
//#define INT_TUNING
//#define DERIV_TUNING
//#define POS_PROP_TUNING
//#define MIN_POW_TUNING
//#define THROTTLE_TUNING
//#define STEER_TUNING
//#define TCR_TUNING

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
#define MIN_POW_MAX 50
#define POS_KP_MAX 1
#define THROTTLE_GAIN_MAX 0.02
#define STEERING_GAIN_MAX 0.5

// Other constants
#define RC_BITS 10
#define TARG_OFFSET 0
#define RAMP_TIME 3e6
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.216
#define MIN_POWER 0.25f

// Globals
unsigned short maxPower = 1024;
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

// PID values
float targetAngle = 0;
float error = 0;
float prevError = 0;
float angle = 0;
float prevAngle;
float proportional = 0;
float integral = 0;
float derivative = 0;
float Kp = 50;
float Ki = 0.11;
float Kd = 0.45;
float posKp = 0;

// P algorithm for speed to determine target angle
int position = 0;
int tempAvPos = 0;
int averagePos = 0;
unsigned int posNum = 0;

// RC control vars
float throttle = 0;
float steering = 0;
float prevThrottle = 0;
float throttleGain = 0.003;
float steeringGain = 0.15;
float throttleOutput = 0;
float calibrationOffset = 0;

// Create encoder motor objects and map ISR array
SpeedMotor m1;
SpeedMotor m2;

// Prototypes
void printData();
bool signToBool(int var);
float inRange(float min_val, float var, float max_val);

// Template for stream
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

HardwareSerial odrive_serial(Serial1);
ODriveArduino odrive(odrive_serial);

// Main Setup
void setup()
{
  Wire.begin();
  Wire.setClock(200000); // 100kHz I2C clock.
  Serial.begin(115200);
  Wire.setTimeOut(100);
  odrive_serial.begin(115200, SERIAL_8N1, ODRIVE_UART_RX, ODRIVE_UART_TX);

  // I2C Setup
  IMUinit();
  RCinit();
  Serial.println(F("Initializing I2C devices..."));
  while (!Serial);
  Serial.println("Serial Ready...");
  while(!odrive_serial);
  Serial.println("Serial1 Ready...");

  //ODrive serial and paramater setup
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 0.8f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 25.0f << '\n';
    //odrive.run_state(axis, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false); // don't wait
  }
}

// Main Loop
void loop() {

    //Sinusoidal test move (only while in velocity control)
    char c = ' ';
    if (Serial.available()) c = Serial.read();
    if (c == 't') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = -ODRIVE_MAX_VELOCITY * sin(ph);
        float pos_m1 = ODRIVE_MAX_VELOCITY * sin(ph);
        odrive.SetVelocity(0, pos_m0);
        odrive.SetVelocity(1, pos_m1);
        delay(10);
      }
    }
    // set axis closed loop state for each motor.
    if (c == '0' || c == '1') {
      int num = c-'0';

      Serial << "Axis" << c << ": Requesting state " << ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL << '\n';
      odrive.run_state(num, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false); // don't wait
    }
    // stop odrive commmunication
    if (c == 's') {
      running = 0;
      Serial.println("Stopped.");
    }
  // Retrieve I2C data
  IMUloop();
  RCloop();

  // Tuning modes
  #if defined(PROP_TUNING)
    Kp = rcAnalogs[2] * KP_MAX / (pow(2,RC_BITS) - 1);  
  #elif defined(INT_TUNING) 
    Ki = rcAnalogs[2] * KI_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(DERIV_TUNING)
    Kd = rcAnalogs[2] * KD_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(MIN_POW_TUNING)
    Kp = 0;
    Ki = 0;
    Kd = 0;
    posKp = 0;
  #elif defined(POS_PROP_TUNING)
    posKp = rcAnalogs[2] * POS_KP_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(THROTTLE_TUNING)
    throttleGain = rcAnalogs[2] * THROTTLE_GAIN_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(TCR_TUNING)
    maxTCR = rcAnalogs[2] * TCR_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(STEER_TUNING)
    steeringGain = rcAnalogs[2] * STEERING_GAIN_MAX / (pow(2,RC_BITS) - 1);
  #endif

  // read encoders and update position
  elapsedTime = esp_timer_get_time();
  unsigned int deltaTime = elapsedTime - prevTime;

  // Throttle Control
  throttle = rcAnalogs[1] * throttleGain;
  steering = rcAnalogs[0] * steeringGain;

  // middle switch for calibrating angle
  if (ch7State == 0) {
    calibrationOffset = (rcAnalogs[3] - 512) * ANGLE_MAX / (pow(2,RC_BITS) - 1);
    throttle = 0;
    steering = 0;
  }

  // get target and current angle in degrees
  targetAngle = calibrationOffset + throttle; // + posProp;
  angle = ypr[2]*180/M_PI;

  // PID for motor output
  error = targetAngle - angle; // convert to degree
  proportional = Kp*error;
  integral += Ki*error;
  integral = inRange(-maxPower, integral, maxPower);
  if (loopCycles % 50 == 0) {
    derivative = Kd*1e6*(error - prevError)/deltaTime;
    prevTime = elapsedTime;
    prevError = error;
  } 
  controlOutput = proportional + integral + derivative;
  short Output1 = controlOutput + steering;
  short Output2 = controlOutput - steering;

  // Kill Switch and Power Ramp Up Logic
  if(ch3State) {
    m1.setPower(0);
    m2.setPower(0);
    killSwitchReleased = 0;
    rampEnabled = 1;
    rampLevel = 0;
  } else {
    if (!killSwitchReleased) {
      rampTime = elapsedTime;
      killSwitchReleased = 1;
      integral = 0;
      m1.resetPosition();
      m2.resetPosition();
    }
    if (rampEnabled) {
      if(elapsedTime - rampTime >= RAMP_TIME) {
        rampLevel = 1;
      }
      else rampLevel = (elapsedTime - rampTime)/ RAMP_TIME;
    }
  }

  // Tune Minimum Power
  #if defined(MIN_POW_TUNING)
    int minPowTest = rcAnalogs[2] * MIN_POW_MAX / (pow(2,RC_BITS) - 1);
    m1.setPower((minPowTest) * rampLevel);
    m2.setPower((minPowTest) * rampLevel);
  #else
    m1.setPower(Output1 * rampLevel);
    m2.setPower(Output2 * rampLevel);
  #endif
  
  // Compensate for minimum power
  if (Output1 >= 0) m1output = float(m1.getPower()*ODRIVE_MAX_TORQUE/maxPower) + MIN_POWER; // scalar is 0.01262
  else m1output = float(m1.getPower()*ODRIVE_MAX_TORQUE/maxPower) - MIN_POWER;
  if (Output2 >= 0) m2output = float(m2.getPower()*ODRIVE_MAX_TORQUE/maxPower) + MIN_POWER;
  else m2output = float(m2.getPower()*ODRIVE_MAX_TORQUE/maxPower) - MIN_POWER;
  
  // Run when r pressed or if running paramater already set
  if (c == 'r' || running) {
      if(odrive_serial.availableForWrite()) {
        odrive.SetCurrent(0, m1output);
        odrive.SetCurrent(1, -m2output);
      }
      running = 1;
  }

  // print data every so often
  loopCycles++;
  if (running) printData();
}

void printData()
{
  if (loopCycles % 100 == 0)
  {
    // Main Loop Cycles
    // Serial.printf("c: %i, ", loopCycles);

    // Motor Data
    Serial.printf("Pwr1: %.2f, ", m1output);
    // Serial.printf("Pos1: %i, ", m1.getPosition());
    // Serial.printf("Pwr2: %i, ", m2.getPower());
    // Serial.printf("Pos2: %i, ", m2.getPosition());

    // IMU Data
    // Serial.printf("roll: %.4f, " ,ypr[2]*180/M_PI);
    // Serial.printf("roll: %.4f, linSpd: %.4f angVel: %.4f, spd %.4f, ", ypr[2]*180/M_PI, linearWheelSpeed ,angularVelocity, speed);
    
    // PID Data
    Serial.printf("gain: %.4f, %.4f, %.4f, ", Kp, Ki, Kd);
    //Serial.printf("PID: %.4f, %.4f, %.4f, ", proportional, integral, derivative);
    
    // RC Data
    // Serial.printf("rc: %i, %i, %i, %i, %i, %i, %i, ", rcAnalogs[0], rcAnalogs[1], rcAnalogs[2], rcAnalogs[3], ch3State, ch4State, ch7State);
    
    // Other Data
    // Serial.printf("targ: %.4f, ", targetAngle);
    // Serial.printf("minPwr: %i, ", m1.getPower());
    // Serial.printf("avPos: %i, posKp: %.4f, ", averagePos, posKp);
    Serial.printf("thrGain: %.6f, strGain: %.6f, thr: %.4f, str: %.4f, ", throttleGain, steeringGain, throttle, steering);
    Serial.printf("ramp: %.4f", rampLevel) ;
  
    Serial.println();
  }
}

float inRange(float min_val, float var, float max_val) {
  if (var < min_val) {
    return min_val;
  } else if (var > max_val) {
    return max_val;
  } else return var;
}

bool signToBool(int var) {
  if (var < 0) return 1;
  else return 0;
}