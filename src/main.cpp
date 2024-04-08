#include <Arduino.h>
#include <EncoderMotor.h>
#include <IMU.h>

// Globals
unsigned int pwmFreq = 5000;
byte pwmResolution = 10;
unsigned short loopCycles = 0;

// Create encoder motor objects and map ISR array
EncoderMotor m1;
volatile bool* interruptArray[2] = {m1.getInterruptA(),m1.getInterruptB()};

// ISR
template <byte i>
void IRAM_ATTR ISR() {
  *interruptArray[i] = 1;
}

// Prototypes
void printData();

void setup() {
  Serial.begin(115200);

  // config for encoder motor
  m1.setupPins(35,34,12,14,27);
  m1.setupPWM(pwmFreq,0,pwmResolution);

  // encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(m1.getEncA()), ISR<0>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1.getEncB()), ISR<1>, CHANGE);

  // set motors to stop
  m1.stopMotor();

  //IMU Setup (refer to IMU.cpp)
  //IMUsetup();
  
  // wait until character is sent before beginning loop
  Serial.println(F("\nSend any character to begin."));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

}

void loop() {
  // put your main code here, to run repeatedly:

  //IMUloop();
  // elapsedTime = esp_timer_get_time();

  // cycle through movements
  m1.setPower(m1.getMaxPower());
  m1.setDirection(0);

  // drive motors
  m1.driveMotor();

  // read encoders and update position
  noInterrupts(); //set interrupts aside
  m1.updatePosition();
  interrupts(); // resume interrupts

  // print data every so often
  printData();
  
}

void printData() {
  if (loopCycles > 200) {
    // Time Data
    // Serial.print("Time: ");
    // Serial.print(elapsedTime);

    // Motor Data
    Serial.print("; M1 pwr: ");
    Serial.print(m1.getPower());
    Serial.print(", dir: ");
    Serial.print(m1.getDirection());
    Serial.print(", pos: ");
    Serial.print(m1.getPosition());

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
  } else {
    loopCycles ++;
  }
}
