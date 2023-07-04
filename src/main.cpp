#include <Arduino.h>
#include <EncoderMotor.h>

unsigned int loopTime;
// PWM setup globals
byte pwmChannel1 = 0;
byte pwmChannel2 = 1;
unsigned int pwmFreq = 50;
byte pwmRes = 10;
unsigned int pwmMax = pow(2,pwmRes) - 1;
unsigned int pwmMin = pwmMax / 8;

// Motor object controls
int pwr1 = 0;
int pwr2 = 0;
byte dir1 = 0;
byte dir2 = 0;

// Motor and driver pins
byte enA = 14;
byte in1 = 27;
byte in2 = 26;
byte in3 = 25;
byte in4 = 33;
byte enB = 32;

// create encoder objects
EncoderMotor e1(35,34);
EncoderMotor e2(39,36);
byte interruptArray[4] = {e1.encAFlag,e1.encBFlag,e2.encAFlag,e2.encBFlag};

// ISR
template <byte i>
void IRAM_ATTR ISR() {
  interruptArray[i] = 1;
}

// Drive motor
void driveMotor(byte pwm_channel, int duty_cycle, byte direction, byte in1_pin, byte in2_pin) {
  ledcWrite(pwm_channel, duty_cycle);
  if (direction == 1) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else {
    digitalWrite(in2_pin, HIGH);
    digitalWrite(in1_pin, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // pwm and direction pin setup
  ledcSetup(pwmChannel1,pwmFreq,pwmRes);
  ledcSetup(pwmChannel2,pwmFreq,pwmRes);
  ledcAttachPin(enA, pwmChannel1);
  ledcAttachPin(enB, pwmChannel2);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // encoder setup
  pinMode(e1.encA, INPUT);
  pinMode(e1.encB, INPUT);
  pinMode(e2.encA, INPUT);
  pinMode(e2.encB, INPUT);
  attachInterrupt(digitalPinToInterrupt(e1.encA), ISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e1.encB), ISR<1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e2.encA), ISR<2>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e2.encB), ISR<3>, CHANGE);

  // set motors to idle
  driveMotor(pwmChannel1, 0, 0, in1, in2);
  driveMotor(pwmChannel2, 0, 0, in3, in4);

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  loopTime = (esp_timer_get_time() / 1000000) % 8; // time in milliseconds (resets every 8 seconds)

  // cycle through movements
  if (loopTime == 0) {
    pwr1 = pwmMax;
    dir1 = 1;
  } else if (loopTime == 2) {
    pwr2 = pwmMax;
    dir2 = 1;
  } else if (loopTime == 4) {
    pwr1 = pwmMin;
    dir1 = 0;
  } else if (loopTime == 6) {
    pwr2 = pwmMin;
    dir2 = 0;
  }
  
  driveMotor(pwmChannel1, pwr1, dir1, in1, in2);
  driveMotor(pwmChannel2, pwr2, dir2, in3, in4);

  // read encoders and update position
  e1.updatePosition(digitalRead(e1.encA), digitalRead(e1.encB));
  e2.updatePosition(digitalRead(e1.encA), digitalRead(e2.encB));

  // print data
  Serial.print("Time: ");
  Serial.print(loopTime);
  Serial.print(", M1 pwm: ");
  Serial.print(pwr1);
  Serial.print(", dir: ");
  Serial.print(dir1);
  Serial.print(", pos: ");
  Serial.print(e1.position);
  Serial.print(", M2 pwm: ");
  Serial.print(pwr2);
  Serial.print(", dir: ");
  Serial.print(dir2);
  Serial.print(", pos: ");
  Serial.print(e2.position);
  Serial.println();
}

