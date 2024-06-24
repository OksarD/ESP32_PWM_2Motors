#include <Arduino.h>
#include <SpeedMotor.h>
#include <ReadI2C.h>
#include <ODriveArduino.h>
#include <SoftwareSerial.h>

//#define PROP_TUNING
//#define INT_TUNING
//#define DERIV_TUNING
//#define POS_PROP_TUNING
//#define MIN_POW_TUNING
#define THROTTLE_TUNING
//#define STEER_TUNING
//#define TCR_TUNING

// Macros
// 27 ESP = 1 ODRV, 26 ESP = 2 ODRV, 25 ESP = 3 ODRV, 33 ESP = 4 ODRV
#define PWR_PIN_1 25
#define CHANNEL_1 0
#define PWR_PIN_2 33
#define CHANNEL_2 1

#define ODRIVE_UART_TX 26
#define ODRIVE_UART_RX 27

#define ANGLE_MAX 4
#define KP_MAX 300
#define KI_MAX 1
#define KD_MAX 10
#define MIN_POW_MAX 50
#define POS_KP_MAX 1
#define TCR_MAX 0.000005
#define THROTTLE_GAIN_MAX 0.02
#define STEERING_GAIN_MAX 0.5

#define RC_BITS 10
#define TARG_OFFSET 0
#define RAMP_TIME 3e6
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.216
#define MIN_POWER 1

// Globals
unsigned int pwmFreq = 5000;
byte pwmResolution = 10;
unsigned short maxPower = pow(2,pwmResolution);
int controlOutput;
bool killSwitchReleased = 1;
unsigned long rampTime = 0;
bool rampEnabled = 0;
float rampLevel = 0;

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
float throttleGain = 0;
float steeringGain = 0.2;
double maxTCR = 0;
float throttleChangeRate = 0;
float throttleOutput = 0;
float calibrationOffset = 0;

// Create encoder motor objects and map ISR array
SpeedMotor m1;
SpeedMotor m2;
volatile bool* interruptArray[2] = {m1.getInterrupt(), m2.getInterrupt()};

// ISR
template <byte i>
void IRAM_ATTR ISR() {
  *interruptArray[i] = 1;
}

// Prototypes
void printData();
bool signToBool(int var);
float inRange(float min_val, float var, float max_val);

// Template for stream
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

SoftwareSerial odrive_serial(27,26);
ODriveArduino odrive(odrive_serial);

// Main Setup
void setup()
{
  Wire.begin();
  Wire.setClock(200000); // 100kHz I2C clock.
  Serial.begin(115200);
  Wire.setTimeOut(100);
  odrive_serial.begin(115200);
  //odrive_serial.begin(115200, SERIAL_8N1, ODRIVE_UART_TX, ODRIVE_UART_RX);

  // config for motors
  m1.setupPins(PWR_PIN_1);
  m1.setupPWM(pwmFreq,CHANNEL_1,pwmResolution);
  m1.setupPins(PWR_PIN_2);
  m1.setupPWM(pwmFreq,CHANNEL_2,pwmResolution);
  
  // kill motors
  ledcWrite(CHANNEL_1, 512);
  ledcWrite(CHANNEL_2, 512);

  // encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(m1.getSC()), ISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2.getSC()), ISR<1>, CHANGE);

  // I2C Setup
  IMUinit();
  RCinit();
  Serial.println(F("Initializing I2C devices..."));
  while (!Serial);
  Serial.println("Serial Ready...");
  while(!odrive_serial);
  Serial.println("Serial1 Ready...");

  //ODrive serial setup
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 25.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }
}

// Main Loop
void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    /* if (c == '0' || c == '1') {
      int motornum = c-'0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n
';
      odrive.run_state(motornum, requested_state, false); // don't wait
    } */

    // Sinusoidal test move
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 50.0f * cos(ph);
        //float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        //odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
  }
  //---------------------------------------
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

  //noInterrupts(); //set interrupts aside
  //m1.update();
  //m2.update();
  //interrupts(); // resume interrupts

  // Throttle Control
  throttle = rcAnalogs[1] * throttleGain;
  steering = rcAnalogs[0] * steeringGain;

  // middle switch states
  if (ch7State == 0) {
    calibrationOffset = (rcAnalogs[3] - 512) * ANGLE_MAX / (pow(2,RC_BITS) - 1);
    throttle = 0;
    steering = 0;
  }

    // throttleChangeRate = throttle - prevThrottle;
    // if (throttleChangeRate > maxTCR) {
    //   throttleOutput = prevThrottle + maxTCR;
    // } else if (throttleChangeRate < -maxTCR) {
    //   throttleOutput = prevThrottle - maxTCR;
    // }
    // prevThrottle = throttle;

  // get target and current angle in degrees
  targetAngle = calibrationOffset + throttle; // + posProp;
  angle = -ypr[2]*180/M_PI;

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

  // Set power and direction
  unsigned int realPower1 = inRange(0, (Output1/2) + (maxPower/2) + MIN_POWER, maxPower);
  unsigned int realPower2 = inRange(0, (Output1/2) + (maxPower/2) + MIN_POWER, maxPower);

  #if defined(MIN_POW_TUNING)
    int minPowTest = rcAnalogs[2] * MIN_POW_MAX / (pow(2,RC_BITS) - 1);
    m1.setPower((minPowTest) * rampLevel);
    m2.setPower((minPowTest) * rampLevel);
  #else
    m1.setPower((realPower1) * rampLevel);
    m2.setPower((realPower2) * rampLevel);
  #endif

  // Drive Motors
  //ledcWrite(CHANNEL_1, m1.getPower());
  //ledcWrite(CHANNEL_2, m2.getPower());

  // print data every so often
  loopCycles++;
  //printData();
}

void printData()
{
  if (loopCycles % 500 == 0)
  {
    // Main Loop Cycles
    // Serial.printf("c: %i, ", loopCycles);
    // Motor Data
    Serial.printf("Pwr1: %i, ", m1.getPower());
    //Serial.printf("Pos1: %i, PCR1: %i, ", m1.getPosition(), m1.getPCR());
    Serial.printf("Pwr2: %i, ", m2.getPower());
    // Serial.printf("Pos2: %i, PCR2: %.2f, ", m2.getPosition(), m2.getPCR());
    // IMU Data
    //Serial.printf("roll: %.4f, " ,ypr[2]*180/M_PI);
    //Serial.printf("roll: %.4f, linSpd: %.4f angVel: %.4f, spd %.4f, ", ypr[2]*180/M_PI, linearWheelSpeed ,angularVelocity, speed);
    // PID Data
    //Serial.printf("gain: %.4f, %.4f, %.4f, ", Kp, Ki, Kd);
    Serial.printf("PID: %.4f, %.4f, %.4f, ", proportional, integral, derivative);
    // RC Data
    // Serial.printf("rc: %i, %i, %i, %i, %i, %i, %i, ", rcAnalogs[0], rcAnalogs[1], rcAnalogs[2], rcAnalogs[3], ch3State, ch4State, ch7State);
    // Other Data
    //Serial.printf("targ: %.4f, ", targetAngle);
    //Serial.printf("minPwr: %i, ", m1.getPower());
    // Serial.printf("avPos: %i, posKp: %.4f, ", averagePos, posKp);
    //Serial.printf("thrGain: %.6f, strGain: %.6f, thr: %.4f, str: %.4f, ", throttleGain, steeringGain, throttle, steering);
    Serial.printf("ramp: %.4f", rampLevel) ;
    // New line
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