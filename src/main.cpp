#include <Arduino.h>
#include <SpeedMotor.h>
#include <ReadI2C.h>

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
#define THROTTLE_GAIN 0.0001
#define STEERING_GAIN 0.005

#define TARG_OFFSET 0.08
#define RAMP_TIME 3e6
#define CENTER_MASS_HEIGHT 0.8
#define PULSES_PER_REV 90
#define WHEEL_DIAMATER 0.216

// Globals
unsigned int pwmFreq = 5000;
byte pwmResolution = 10;
unsigned short maxPower = pow(2,pwmResolution) - 1;
int controlOutput;
unsigned int minPower = 0;
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
float Kp = 70;
float Ki = 0;
float Kd = 0;

// P algorithm for speed to determine target angle
float speed;
float targetSpeed = 0;
float speedError = 0;
float speedKp = 0.05;
float angularVelocity = 0;
float linearWheelSpeed = 0;

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
  #if defined(PROP_ANGLE_TUNING)
    Kp = rcAnalogs[2] * KP_MAX / (pow(2,RC_BITS) - 1);
    if (ch7State == 1) targetAngle = (rcAnalogs[3] - 512) * ANGLE_MAX / (pow(2,RC_BITS) - 1);
    else targetAngle = TARG_OFFSET;
  #elif defined(INT_DERIV_TUNING) 
    Ki = rcAnalogs[2] * KI_MAX / (pow(2,RC_BITS) - 1);
    Kd = rcAnalogs[3] * KD_MAX / (pow(2,RC_BITS) - 1);
  #endif

  // read encoders and update position
  elapsedTime = esp_timer_get_time();
  unsigned int deltaTime = elapsedTime - prevTime;
  noInterrupts(); //set interrupts aside
  m1.update();
  m2.update();
  interrupts(); // resume interrupts

  // calculate horizontal speed at center of mass
  angle = -ypr[2];
  if (loopCycles % 500 == 0) {
    float averageMotorSpeed = (m1.getSpeed() + m2.getSpeed()) / 2;
    linearWheelSpeed = averageMotorSpeed*WHEEL_DIAMATER/PULSES_PER_REV;
    angularVelocity = (angle - prevAngle)*1e6/deltaTime;
    float relativeBodySpeed = cos(angle)*angularVelocity*CENTER_MASS_HEIGHT/2;
    speed = relativeBodySpeed + linearWheelSpeed;
    prevAngle = angle;
  }

  // P for target angle
  speedError = targetSpeed - speed;
  //targetAngle = (speedKp * speedError);
  targetAngle = rcAnalogs[1] * THROTTLE_GAIN;
  // PID for motor output
  error = (targetAngle - angle) *180/M_PI;
  proportional = Kp*error;
  integral += Ki*error;
  integral = inRange(-maxPower, integral, maxPower);
  if (loopCycles % 50 == 0) {
    derivative = Kd*1e6*(error - prevError)/(deltaTime);
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
    }
    if (rampEnabled) {
      if(elapsedTime - rampTime >= RAMP_TIME) {
        rampLevel = 1;
      }
      else rampLevel = (elapsedTime - rampTime)/ RAMP_TIME;
    }
  }

  // Set power and direction
  unsigned int realPower = inRange(-maxPower, abs(controlOutput), maxPower);
  bool dir = signToBool(controlOutput);
  m1.setPower(realPower * rampLevel);
  m1.setDirection(dir);
  m2.setPower(realPower * rampLevel);
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
    Serial.printf("Pwr1: %i, Dir1: %i, Brk1: %i ", m1.getPower(), m1.getDirection(), m1.getBrake());
    // Serial.printf("Pos1: %i, PCR1: %i, ", m1.getPosition(), m1.getPCR());
    Serial.printf("Pwr2: %i, Dir2: %i, Brk2: %i ", m2.getPower(), m2.getDirection(), m2.getBrake());
    // Serial.printf("Pos2: %i, PCR2: %.2f, ", m2.getPosition(), m2.getPCR());
    // IMU Data
    // Serial.printf("roll: %.4f, linSpd: %.4f angVel: %.4f, spd %.4f, ", ypr[2]*180/M_PI, linearWheelSpeed ,angularVelocity, speed);
    // PID Data
    // Serial.printf("gain: %.4f, %.4f, %.4f, ", Kp, Ki, Kd);
    Serial.printf("PID: %.4f, %.4f, %.4f, ", proportional, integral, derivative);
    // RC Data
    // Serial.printf("rc: %i, %i, %i, %i, %i, %i, %i, ", rcAnalogs[0], rcAnalogs[1], rcAnalogs[2], rcAnalogs[3], ch3State, ch4State, ch7State);
    // Other Data
    Serial.printf("targ: %.4f, ", targetAngle *180/M_PI);
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