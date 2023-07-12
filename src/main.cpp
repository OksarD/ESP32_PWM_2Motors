#include <Arduino.h>
#include <EncoderMotor.h>
#include <IMU.h>

unsigned int loopTime;
unsigned long elapsedTime;
unsigned short int loopCycles = 0;

// PWM Globals
unsigned int pwmMinimumFreq = 50;
unsigned int pwmMaximumFreq = 1000;
byte pwmResolution = 10;

// create encoder motor objects and map ISR array
EncoderMotor m1;
EncoderMotor m2;
volatile byte* interruptArray[4] = {m1.interruptA,m1.interruptB,m2.interruptA,m2.interruptB};

// ISR
template <byte i>
void IRAM_ATTR ISR() {
  *interruptArray[i] = 1;
}

// Declare Serial printing function (definition underneath main loop)
void printData();

void setup() {
  Serial.begin(115200);

  // config for encoder motor
  m1.setupPins(35,34,14,27,26);
  m1.setupPWM(pwmMinimumFreq,pwmMaximumFreq,0,pwmResolution);
  m2.setupPins(39,36,32,25,33);
  m2.setupPWM(pwmMinimumFreq,pwmMaximumFreq,1,pwmResolution);

  // encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(m1.encA), ISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1.encB), ISR<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2.encA), ISR<2>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2.encB), ISR<3>, CHANGE);

  // set motors to stop
  m1.driveMotor(0,0);
  m2.driveMotor(0,0);

  //IMU Setup (refer to IMU.cpp)
  IMUsetup();
  
  // wait until character is sent before beginning loop
  Serial.println(F("\nSend any character to begin."));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

}

void loop() {
  // put your main code here, to run repeatedly:

  IMUloop();

  elapsedTime = esp_timer_get_time();
  loopTime = (elapsedTime / 1000) % 8000; // time in milliseconds (resets every 8 seconds)
  unsigned int loopTimeSec = loopTime / 1000;
  // cycle through movements
  if (loopTimeSec == 2) {
    m1.setPower(m1.getMaxPower());
    m1.setDirection(1);
    m2.setPower(m2.getMaxPower());
    m2.setDirection(1);
  } else if (loopTimeSec == 6) {
    m2.setPower(m2.getMaxPower() * 0.25);
    m2.setDirection(0);
    m1.setPower(m1.getMaxPower() * 0.25);
    m1.setDirection(0);
  }
  

  /*
  unsigned int sawtooth = (loopTime * (m1.getMaxPower() - 50) / 8000) + 15;
  unsigned int wavysaw = (30 * sin(float(loopTime) / float(100))) + (loopTime * (m1.getMaxPower() - 50) / 8000) + 15;
  m1.setPower(sawtooth);
  m1.setDirection(0);
  m2.setPower(wavysaw);
  m2.setDirection(1);
  */

  // drive motors
  m1.driveMotor();
  m2.driveMotor();

  noInterrupts(); //set interrupts aside

  // read encoders and update position
  m1.updatePosition();
  m2.updatePosition();

  interrupts(); // resume interrupts

  // print data every 1000 cycles of code
  printData();
  
}

void printData() {
  if (loopCycles > 200) {
    // Time Data
    // Serial.print("Time: ");
    // Serial.print(elapsedTime);

    // Motor Data
    // Serial.print("; M1 pwm: ");
    // Serial.print(m1.power);
    // Serial.print(", dir: ");
    // Serial.print(m1.direction);
    // Serial.print(", freq: ");
    // Serial.print(m1.frequency);
    // Serial.print(", pos: ");
    // Serial.print(m1.position);

    // Serial.print("; M2 pwm: ");
    // Serial.print(m2.power);
    // Serial.print(", dir: ");
    // Serial.print(m2.direction);
    // Serial.print(", freq: ");
    // Serial.print(m2.frequency);
    // Serial.print(", pos: ");
    // Serial.print(m2.position);

    // IMU Data
    Serial.print(", ypr: ");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print(", ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(", ");
    Serial.print(ypr[2] * 180/M_PI);

    // Serial.print("; areal: ");
    // Serial.print(aaReal.x);
    // Serial.print(", ");
    // Serial.print(aaReal.y);
    // Serial.print(", ");
    // Serial.print(aaReal.z);

    // New line
    Serial.println();
    loopCycles = 0;
  } else {
    loopCycles ++;
  }
}
