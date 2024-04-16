#include <Arduino.h>
#include <SpeedMotor.h>
#include <IMU.h>
#include <MotorPID.h>

// Macros
#define SC_PIN 19
#define PWR_PIN 26
#define DIR_PIN 25
#define BRAKE_PIN 33

// Globals
unsigned short loopCycles = 0;

unsigned int pwmFreq = 5000;
byte pwmResolution = 10;
unsigned long elapsedTime = 0;
int motorPower = 0;

// Create encoder motor objects and map ISR array
SpeedMotor m1;
volatile bool* interruptArray[1] = {m1.getInterrupt()};

// ISR
template <byte i>
void IRAM_ATTR ISR()
{
  *interruptArray[i] = 1;
}

// Prototypes
void printData();
void boolInvert(bool var);
bool signToBool(int var);

// Main Setup
void setup()
{
  Serial.begin(115200);

  // config for encoder motor
  m1.setupPins(SC_PIN, PWR_PIN, DIR_PIN, BRAKE_PIN);
  m1.setupPWM(pwmFreq,0,pwmResolution);

  // encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(m1.getSC()), ISR<0>, CHANGE);

  // start Motor
  m1.stopMotor();

  // IMU Setup
  // IMUsetup();

  // wait until character is sent before beginning loop
  Serial.println(F("\nSend any character to begin."));

}

// Main Loop
void loop()
{
  elapsedTime = esp_timer_get_time();
  //IMUloop();

  // cycle through movements
  motorPower = -m1.getMaxPower()*sin((2*PI*elapsedTime/(1e6*20)));
  m1.setPower(abs(motorPower));
  m1.setDirection(signToBool(motorPower));
  m1.driveMotor();
  // read encoders and update position
  noInterrupts(); //set interrupts aside
  m1.update();
  interrupts(); // resume interrupts

  // print data every so often
  printData();
}

void printData()
{
  if (loopCycles > 5000)
  {
    Serial.print("Time: ");
    Serial.print(esp_timer_get_time());

    // Motor Data
    Serial.print(", Pwr: ");
    Serial.print(m1.getPower());
    Serial.print(", Dir: ");
    Serial.print(m1.getDirection());
    Serial.print(", Pos: ");
    Serial.print(m1.getPosition());
    Serial.print(", RPM: ");
    Serial.print(m1.getSpeed()*0.6667);
    // IMU Data
    // Serial.print(", ypr: ");
    // Serial.print(ypr[0] * 180/M_PI);
    // Serial.print(", ");
    // Serial.print(ypr[1] * 180/M_PI);
    // Serial.print(", ");
    // Serial.print(ypr[2] * 180/M_PI);

    // Serial.print("; areal: ");
    // Serial.print(aaReal.x);
    // Serial.print(", ");
    // Serial.print(aaReal.y);
    // Serial.print(", ");
    // Serial.print(aaReal.z);

    // New line
    Serial.println();
    loopCycles = 0;
  }
  else
  {
    loopCycles++;
  }
}

void boolInvert(bool var) {
    if (var) var = 0;
    else var = 1;
}

bool signToBool(int var) {
  if (var < 0) return 1;
  else return 0;
}