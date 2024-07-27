#include <Arduino.h>
#include <SpeedMotor.h>
#include <ReadI2C.h>
#include "ODriveArduino.h"
#include <Control.h>
#include <Macros.h>

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
float Ki = 0.12;
float Kd = 5;

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
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 50.0f << '\n';
  }

  pinMode(LED_PIN, OUTPUT);
}

// Main Loop
void loop() {

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
  #elif defined(POS_PROP_TUNING)
    posKp = rcAnalogs[2] * POS_KP_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(THROTTLE_TUNING)
    throttleGain = rcAnalogs[2] * THROTTLE_GAIN_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(STEER_PROP_TUNING)
    steerKp = rcAnalogs[2] * STEER_KP_MAX / (pow(2,RC_BITS) - 1);
  #elif defined(STEER_INT_TUNING)
    steerKi = rcAnalogs[2] * STEER_INT_MAX / (pow(2,RC_BITS) - 1);
  #endif

  // read encoders and update position
  elapsedTime = esp_timer_get_time();

  // Throttle Control
  throttle = rcAnalogs[1] * throttleGain;
  steering = rcAnalogs[0] * 1*ODRIVE_MAX_VELOCITY / MAX_POWER;

  // switch CH7 left for angle tuning, left for motor calibration, middle for operation
  if (ch7State == 0) {
    calibrationOffset = (rcAnalogs[3] - 512) * ANGLE_MAX / (pow(2,RC_BITS) - 1);
    throttle = 0;
    steering = 0;
  } 
  #if defined(ENABLE_MOTOR_CALIBRATION) 
  if (ch7State == 2) {
    if (!motor_calib_state) {
      motor_calib_state = 1;
      for (int num = 0; num < 2; num++) {
        Serial.printf("Calibrating motor %i...\n", num);
        odrive.run_state(num, ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION, false); // don't wait
        delay(5000);
        odrive.run_state(num, ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION, false); // don't wait
        delay(5000);
      }
      delay(22000);
      Serial.println("Done motor calibration.");
      odrive.run_state(0, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false); // don't wait
      odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false); // don't wait
    }
  } else if {
    motor_calib_state = 0
  }
  #endif
  
  // get target and current angle in degrees
  targetAngle = calibrationOffset + throttle; // + posProp;
  angle = ypr[2]*180/M_PI;
  // get motor speeds
  
  if (loopCycles % 20 == 0) {
    odrive_serial << "r axis0.encoder.vel_estimate\n";
    m1.setSpeed(odrive.readFloat());
  } else if ((loopCycles + 10) % 20 == 0) {
    odrive_serial << "r axis1.encoder.vel_estimate\n";
    m2.setSpeed(-odrive.readFloat());
  }
  
  // PI algorithm to control steering
  speedDiff = -m2.getSpeed() + m1.getSpeed();
  steeringError = steering - speedDiff;
  steerProportional = steerKp*steeringError;
  steerIntegral += steerKi*steeringError;
  steerIntegral = inRange(-MAX_POWER, steerIntegral, MAX_POWER);
  steerOutput = steerProportional + steerIntegral;

  BalanceLoop();
  m1.setPower(controlOutput + steerOutput);
  m2.setPower(controlOutput - steerOutput);

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

  // Compensate for minimum power and scale
  if (m1.getPower() >= 0) m1output = (m1.getPower()*ODRIVE_MAX_TORQUE/MAX_POWER) + MIN_POWER_1; // scalar is 0.01262
  else m1output = float(m1.getPower()*ODRIVE_MAX_TORQUE/MAX_POWER) - MIN_POWER_1;
  if (m2.getPower() >= 0) m2output = float(m2.getPower()*ODRIVE_MAX_TORQUE/MAX_POWER) + MIN_POWER_2;
  else m2output = float(m2.getPower()*ODRIVE_MAX_TORQUE/MAX_POWER) - MIN_POWER_2;
  
  // Keyboard input controls
  char c = ' ';
  if (Serial.available()) c = Serial.read();
  // Run when r pressed or if running paramater already set
  if (c == 'r' || running) {
      if(odrive_serial.availableForWrite()) {
        odrive.SetCurrent(1, -m2output* rampLevel);
        odrive.SetCurrent(0, m1output* rampLevel);
      }
      running = 1;
  }
  // Sinusoidal test move (only while in velocity control)
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
  // stop odrive commmunication
  if (c == 's') {
    running = 0;
    Serial.println("Stopped.");
  }

  // print data every so often
  loopCycles++;
  if (running) printData();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void printData()
{
  if (loopCycles % 100 == 0)
  {
    // Main Loop Cycles
    // Serial.printf("c: %i, ", loopCycles);

    // Motor Data
    Serial.printf("Pwr1: %.2f, Out1: %.2f ", m1.getPower(), m1output*rampLevel);
    Serial.printf("Spd1: %.2f, ", m1.getSpeed());
    //Serial.printf("Pwr2: %.2f, Out2: %.2f ", m2.getPower(), m2output*rampLevel);
    // Serial.printf("Spd2: %.2f, ", m2.getSpeed());
    Serial.printf("Diff: %.2f, ", speedDiff);
    // IMU Data
    //Serial.printf("roll: %.4f, trim: %.2f" ,ypr[2]*180/M_PI, calibrationOffset);
    
    // PID Data
    //Serial.printf("gain: %.4f, %.4f, %.4f, ", Kp, Ki, Kd);
    //Serial.printf("PID: %.4f, %.4f, %.4f, ", proportional, integral, derivative);
    Serial.printf("StrGain: %.2f, %.2f, ", steerKp, steerKi);
    Serial.printf("StrPI: %.2f, %.2f, ", steerProportional, steerIntegral);
    Serial.printf("StrErr: %.2f, ", steeringError);
    // RC Data
    // Serial.printf("rc: %i, %i, %i, %i, %i, %i, %i, ", rcAnalogs[0], rcAnalogs[1], rcAnalogs[2], rcAnalogs[3], ch3State, ch4State, ch7State);
    
    // Other Data
    // Serial.printf("targ: %.4f, ", targetAngle);
    // Serial.printf("minPwr: %i, ", m1.getPower());
    // Serial.printf("avPos: %i, posKp: %.4f, ", averagePos, posKp);
    //Serial.printf("thrGain: %.6f, strGain: %.6f, ", throttleGain, steeringGain);
    Serial.printf("thr: %.4f, str: %.4f, ", throttle, steering);
    //Serial.printf("ramp: %.4f", rampLevel) ;
  
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