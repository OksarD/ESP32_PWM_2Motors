#include <Arduino.h>
#include <SpeedMotor.h>
#include <ReadI2C.h>
#include <MotorPID.h>

//#define PROP_ANGLE_TUNING
#define INT_DERIV_TUNING

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

#define ANGLE_MAX 2
#define KP_MAX 300
#define KI_MAX 1
#define KD_MAX 10
#define RC_BITS 10

#define TARG_OFFSET 0.08
#define RAMP_TIME 3e6

// Globals
unsigned short loopCycles = 0;
unsigned int pwmFreq = 5000;
byte pwmResolution = 10;
unsigned short maxPower = pow(2,pwmResolution) - 1;
unsigned long elapsedTime = 0;
unsigned long prevTime = 0;
int motorPower;
unsigned int minPower = 0;
bool killSwitchReleased = 1;
bool rampEnabled = 0;
float rampLevel = 0;
// PID values
float targetAngle = TARG_OFFSET;
float error = 0;
float prevError = 0;
float angle = 0;
float proportional = 0;
float integral = 0;
float derivative = 0;
float Kp = 100;
float Ki = 0;
float Kd = 0;

// P algorithm for position
int Pos[2] = {0,0};
int targetPos[2] = {0,0};
int posError[2] = {0,0};
int posProp[2] = {0,0};
float PosKp = 0.05;

// Create encoder motor objects and map ISR array
SpeedMotor m1;
SpeedMotor m2;
volatile bool* interruptArray[2] = {m1.getInterrupt(), m2.getInterrupt()};

// ISR
template <byte i>
void IRAM_ATTR ISR()
{
  *interruptArray[i] = 1;
}

// Prototypes
void printData();
bool signToBool(int var);
float inRange(float min_val, float var, float max_val);

auto rampTimer = timerBegin(0, 80, true);

// Main Setup
void setup()
{
  Wire.begin();
  Wire.setClock(200000); // 100kHz I2C clock.
  Serial.begin(115200);
  Wire.setTimeOut(100);

  // Ramp Up timer
  timerAlarmWrite(rampTimer, RAMP_TIME, true);

  // config for encoder motor
  m1.setupPins(SC_PIN_1, PWR_PIN_1, DIR_PIN_1, BRK_PIN_1);
  m1.setupPWM(pwmFreq,CHANNEL_1,pwmResolution);
  m1.setupPins(SC_PIN_2, PWR_PIN_2, DIR_PIN_2, BRK_PIN_2);
  m1.setupPWM(pwmFreq,CHANNEL_2,pwmResolution);
  
  // set motors to zero
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
  IMUloop();
  RCloop();

  // Kill Switch and Ramp Up
  
  if(ch3State) {
    digitalWrite(BRK_PIN_1, HIGH);
    digitalWrite(BRK_PIN_2, HIGH);
    timerAlarmDisable(rampTimer);
    killSwitchReleased = 0;
    rampEnabled = 1;
    rampLevel = 0;
  } else {
    if (!killSwitchReleased) {
      timerAlarmEnable(rampTimer);
      killSwitchReleased = 1;
    }
    digitalWrite(BRK_PIN_1, LOW);
    digitalWrite(BRK_PIN_2, LOW);
    if (rampEnabled) {
      int currentRampTime = timerReadMicros(rampTimer);
      if(currentRampTime >= RAMP_TIME) {
        timerAlarmDisable(rampTimer);
        rampLevel = 1;
      }
      else rampLevel = currentRampTime / RAMP_TIME;
    }
  }
  // Tuning modes
  #if defined(PROP_ANGLE_TUNING)
    Kp = rcAnalogs[2] * KP_MAX / (pow(2,RC_BITS) - 1);
    if (ch7State == 1) targetAngle = (rcAnalogs[3] - 512) * ANGLE_MAX / (pow(2,RC_BITS) - 1);
    else targetAngle = TARG_OFFSET;
  #elif defined(INT_DERIV_TUNING) 
    Ki = rcAnalogs[2] * KI_MAX / (pow(2,RC_BITS) - 1);
    Kd = rcAnalogs[3] * KD_MAX / (pow(2,RC_BITS) - 1);
  #endif

  // P for target angle
  Pos[0] = m1.getPosition();
  Pos[1] = m2.getPosition();
  for (int i = 0; i < 2; i++) {
    posError[i] = targetPos[i] - Pos[i];
    posProp[i] = PosKp * posError[i];
  }
  //targetAngle = (posProp[0] + posProp[1])/2;

  // PID for motor output
  angle = -ypr[2]*180/PI;
  error = targetAngle - angle;
  integral += Ki*error;
  integral = inRange(-maxPower, integral, maxPower);
  proportional = Kp*error;
  if (loopCycles % 50 == 0) {
    elapsedTime = esp_timer_get_time();
    derivative = Kd*1e6*(error - prevError)/(elapsedTime - prevTime);
    // Serial.printf("DT: %i, DE: %.4f, ", elapsedTime - prevTime, error - prevError);
    prevTime = elapsedTime;
    prevError = error;
  } 
  motorPower = proportional + integral + derivative;
  // motorPower = -maxPower*sin((2*PI*elapsedTime/(1e6*20)));

  // Set power, including exclusion zone
  unsigned int realPower = inRange(0, abs(motorPower), maxPower-minPower) + minPower;
  bool dir = signToBool(motorPower);
  m1.setPower(realPower * rampLevel);
  m1.setDirection(dir);
  m2.setPower(realPower * rampLevel);
  m2.setDirection(dir);

  // Drive Motors
  ledcWrite(CHANNEL_1, m1.getPower());
  ledcWrite(CHANNEL_2, m2.getPower());
  if (m1.getDirection()) digitalWrite(DIR_PIN_1, LOW);
  else digitalWrite(DIR_PIN_1, HIGH);
  if (m2.getDirection()) digitalWrite(DIR_PIN_2, HIGH);
  else digitalWrite(DIR_PIN_2, LOW);

  // read encoders and update position
  noInterrupts(); //set interrupts aside
  m1.update();
  m2.update();
  interrupts(); // resume interrupts

  // print data every so often
  printData();
  loopCycles ++;
}

void printData()
{
  if (loopCycles % 50 == 0)
  {
    // Main Loop Cycles
    // Serial.printf("c: %i, ", loopCycles);
    // Motor Data
    Serial.printf("Pwr1: %i, Dir1: %i, ", m1.getPower(), m1.getDirection());
    // Serial.printf("Pos2: %i, RPM2: %.2f, ", m2.getPosition(), m2.getSpeed()*0.6667);
    // Serial.printf("Pwr2: %i, Dir2: %i, ", m2.getPower(), m2.getDirection());
    // Serial.printf("Pos1: %i, RPM1: %.2f, ", m1.getPosition(), m1.getSpeed()*0.6667);
    // IMU Data
    Serial.printf("ypr: %.4f, %.4f, %.4f, ", ypr[0]*180/M_PI, ypr[1]*180/M_PI, ypr[2]*180/M_PI);
    // Serial.printf("acc: %.4f, %.4f, %.4f, ", aaReal.x, aaReal.y, aaReal.z);
    // PID Data
    Serial.printf("gain: %.4f, %.4f, %.4f, ", Kp, Ki, Kd);
    Serial.printf("PID: %.4f, %.4f, %.4f, ", proportional, integral, derivative);
    // RC Data
    // Serial.printf("rc: %i, %i, %i, %i, %i, %i, %i, ", rcAnalogs[0], rcAnalogs[1], rcAnalogs[2], rcAnalogs[3], ch3State, ch4State, ch7State);
    // Other Data
    // Serial.printf("targ: %.4f, ", targetAngle);
    // Serial.printf("ramp: %.4f, %i", rampLevel, timerRead(rampTimer)) ;
    // New line
    Serial.println();
  }
}

bool signToBool(int var) {
  if (var < 0) return 1;
  else return 0;
}

float inRange(float min_val, float var, float max_val) {
  if (var < min_val) {
    return min_val;
  } else if (var > max_val) {
    return max_val;
  } else return var;
}