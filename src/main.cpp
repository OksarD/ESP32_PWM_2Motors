#include <Arduino.h>
#include <SpeedMotor.h>
#include <ReadI2C.h>

//#define PROP_TUNING
//#define INT_TUNING
//#define DERIV_TUNING
#define POS_PROP_TUNING
//#define MIN_POW_TUNING

// Macros
#define SC_PIN_1 19
#define PWR_PIN_1 26
#define DIR_PIN_1 25
#define BRK_PIN_1 33
#define CHANNEL_1 0
#define SC_PIN_2 18
#define PWR_PIN_2 13
#define DIR_PIN_2 14
#define BRK_PIN_2 27
#define CHANNEL_2 1

#define ANGLE_MAX 4 * (M_PI/180)
#define KP_MAX 300
#define KI_MAX 1
#define KD_MAX 10
#define MIN_POW_MAX 50
#define POS_KP_MAX 1

#define RC_BITS 10
#define THROTTLE_GAIN 0.0001
#define STEERING_GAIN 0.005

#define TARG_OFFSET 0
#define RAMP_TIME 3e6
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.216
#define MIN_POWER 20

// Globals
unsigned int pwmFreq = 5000;
byte pwmResolution = 10;
unsigned short maxPower = pow(2,pwmResolution) - 1 - MIN_POWER;
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
float Kp = 65;
float Ki = 0.140;
float Kd = 0.6;
float posKp = 0;

// P algorithm for speed to determine target angle
int position = 0;
int tempAvPos = 0;
int averagePos = 0;
unsigned int posNum = 0;

// RC control vars
float throttle;
float prevThrottle;

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

hw_timer_t* rampTimer = timerBegin(0, 80, true);

// Main Setup
void setup()
{
  Wire.begin();
  Wire.setClock(200000); // 100kHz I2C clock.
  Serial.begin(115200);
  Wire.setTimeOut(100);

  // Ramp Up timer
  //timerAlarmWrite(rampTimer, RAMP_TIME, true);

  // config for motors
  m1.setupPins(SC_PIN_1, PWR_PIN_1, DIR_PIN_1, BRK_PIN_1);
  m1.setupPWM(pwmFreq,CHANNEL_1,pwmResolution);
  m1.setupPins(SC_PIN_2, PWR_PIN_2, DIR_PIN_2, BRK_PIN_2);
  m1.setupPWM(pwmFreq,CHANNEL_2,pwmResolution);
  
  // kill motors
  ledcWrite(CHANNEL_1, 0);
  ledcWrite(CHANNEL_2, 0);
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, HIGH);
  digitalWrite(BRK_PIN_1, HIGH);
  digitalWrite(BRK_PIN_2, HIGH);

  // encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(m1.getSC()), ISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2.getSC()), ISR<1>, CHANGE);

  // I2C Setup
  IMUinit();
  RCinit();
  Serial.println(F("Initializing I2C devices..."));
  while (!Serial);
}

// Main Loop
void loop()
{
  // Retrieve I2C data
  IMUloop();
  RCloop();

  // Tuning modes
  float calibrationOffset;
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
  #endif
  // Angle Tuning
  if (ch7State == 1) calibrationOffset = (rcAnalogs[3] - 512) * ANGLE_MAX / (pow(2,RC_BITS) - 1);
  else targetAngle = TARG_OFFSET;

  // read encoders and update position
  elapsedTime = esp_timer_get_time();
  unsigned int deltaTime = elapsedTime - prevTime;
  noInterrupts(); //set interrupts aside
  m1.update();
  m2.update();
  interrupts(); // resume interrupts

  // P for target angle
  float posProp = 0;
  // position = (m1.getPosition() + m2.getPosition()) * 0.5;
  // if (posNum < 100) {
  //   tempAvPos += position;
  //   posNum ++;
  // } else if (posNum == 100) {
  //   tempAvPos /= 100;
  //   averagePos = tempAvPos;
  //   tempAvPos = 0;
  //   posNum = 0;

  //   posProp = averagePos * posKp;
  // }
  
  targetAngle = calibrationOffset + posProp;

  // PID for motor output
  error = (targetAngle - angle) *180/M_PI;
  proportional = Kp*error;
  integral += Ki*error;
  integral = inRange(-maxPower, integral, maxPower);
  if (loopCycles % 50 == 0) {
    derivative = Kd*1e6*(error - prevError)/deltaTime;
    prevTime = elapsedTime;
    prevError = error;
  } 
  controlOutput = proportional + integral + derivative;

  // Kill Switch and Power Ramp Up Logic
  if(ch3State) {
    m1.setBrake(1);
    m2.setBrake(1);
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
  unsigned int realPower = inRange(-maxPower, abs(controlOutput), maxPower) + MIN_POWER;
  bool dir = signToBool(controlOutput);
  #if defined(MIN_POW_TUNING)
    int minPowTest = rcAnalogs[2] * MIN_POW_MAX / (pow(2,RC_BITS) - 1);
    m1.setPower((minPowTest) * rampLevel);
    m2.setPower((minPowTest) * rampLevel);
  #else
    m1.setPower((realPower) * rampLevel);
    m2.setPower((realPower) * rampLevel);
  #endif
  m1.setDirection(dir);
  m2.setDirection(dir);



  // Drive Motors
  ledcWrite(CHANNEL_1, abs(m1.getPower()));
  ledcWrite(CHANNEL_2, abs(m2.getPower()));
  digitalWrite(DIR_PIN_1, !m1.getDirection());
  digitalWrite(DIR_PIN_2, m2.getDirection());
  digitalWrite(BRK_PIN_1, m1.getBrake());
  digitalWrite(BRK_PIN_2, m2.getBrake());

  // print data every so often
  loopCycles++;
  printData();
}

void printData()
{
  if (loopCycles % 500 == 0)
  {
    // Main Loop Cycles
    // Serial.printf("c: %i, ", loopCycles);
    // Motor Data
    //Serial.printf("Pwr1: %i, Dir1: %i, Brk1: %i ", m1.getPower(), m1.getDirection(), m1.getBrake());
    Serial.printf("Pos1: %i, PCR1: %i, ", m1.getPosition(), m1.getPCR());
    //Serial.printf("Pwr2: %i, Dir2: %i, Brk2: %i ", m2.getPower(), m2.getDirection(), m2.getBrake());
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
    Serial.printf("targ: %.4f, ", targetAngle *180/M_PI);
    //Serial.printf("minPwr: %i, pwr: %i, ", m1.getPower(), controlOutput);
    Serial.printf("avPos: %i, posKp: %.4f, ", averagePos, posKp);
    // Serial.printf("ramp: %.4f", rampLevel) ;
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