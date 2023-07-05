#include <Arduino.h>
#include <EncoderMotor.h>

unsigned int loopTime;
unsigned long elapsedTime;

// PWM Globals
unsigned int pwmMinimumFreq = 50;
unsigned int pwmMaximumFreq = 500;
float DynamicFreqPowerThreshold = 0.5;
byte pwmResolution = 10;

// create encoder motor objects and map ISR array
EncoderMotor m1;
EncoderMotor m2;
byte interruptArray[4] = {m1.encAFlag,m1.encBFlag,m2.encAFlag,m2.encBFlag};

// ISR
template <byte i>
void IRAM_ATTR ISR() {
  interruptArray[i] = 1;
}

void setup() {
  Serial.begin(115200);

  // config for encoder motor
  m1.configurePins(35,34,14,27,26);
  m1.configurePWM(pwmMinimumFreq,pwmMaximumFreq,DynamicFreqPowerThreshold,0,pwmResolution);
  m2.configurePins(39,36,32,25,33);
  m2.configurePWM(pwmMinimumFreq,pwmMaximumFreq,DynamicFreqPowerThreshold,1,pwmResolution);

  // encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(m1.encA), ISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1.encB), ISR<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2.encA), ISR<2>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2.encB), ISR<3>, CHANGE);

  // set motors to stop
  m1.driveMotor(0,0);
  m2.driveMotor(0,0);

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  elapsedTime = esp_timer_get_time();
  loopTime = (elapsedTime / 1000) % 8000; // time in milliseconds (resets every 8 seconds)

  // cycle through movements
  /*
  if (loopTime == 0) {
    m1.setPower(m1.getMaxPower() * 0.5);
    m1.setDirection(1);
  } else if (loopTime == 2000) {
    m2.setPower(m2.getMaxPower() * 0.4);
    m2.setDirection(1);
  } else if (loopTime == 4000) {
    m1.setPower(m1.getMaxPower() * 0.3);
    m1.setDirection(0);
  } else if (loopTime == 6000) {
    m2.setPower(m2.getMaxPower() * 0.2);
    m2.setDirection(0);
  }
  */

  unsigned int sawtooth = (loopTime * (m1.getMaxPower() - 50) / 8000) + 15;
  unsigned int wave = (30 * sin(float(loopTime) / float(100))) + 15;
  unsigned int wavysaw = (30 * sin(float(loopTime) / float(100))) + (loopTime * (m1.getMaxPower() - 50) / 8000) + 15;
  m1.setPower(sawtooth);
  m1.setDirection(0);
  m2.setPower(wavysaw);
  m2.setDirection(1);

  // drive motors
  m1.driveMotor();
  m2.driveMotor();

  // read encoders and update position
  m1.updatePosition();
  m2.updatePosition();

  // print data
  Serial.print("Time: ");
  Serial.print(loopTime);
  Serial.print(", M1 pwm: ");
  Serial.print(m1.power);
  Serial.print(", dir: ");
  Serial.print(m1.direction);
  Serial.print(", freq: ");
  Serial.print(m1.frequency);
  Serial.print(", pos: ");
  Serial.print(m1.position);
  Serial.print(", M2 pwm: ");
  Serial.print(m2.power);
  Serial.print(", dir: ");
  Serial.print(m2.direction);
  Serial.print(", freq: ");
  Serial.print(m2.frequency);
  Serial.print(", pos: ");
  Serial.print(m2.position);
  Serial.println();
}

