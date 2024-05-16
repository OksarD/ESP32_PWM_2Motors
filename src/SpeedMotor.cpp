#include "SpeedMotor.h"

unsigned long elapsedTime = 0;
unsigned long prevTime = 0;
unsigned short loopCycles = 0;

bool signToBool(int var) {
  if (var < 0) return 1;
  else return 0;
}

// Configure Pins for Motor
void SpeedMotor::setupPins(byte sc_pin, byte pwr_pin, byte dir_pin, byte brake_pin) {
    scPin = sc_pin;
    enable = pwr_pin;
    dirPin = dir_pin;
    brakePin = brake_pin;
    pinMode(scPin, INPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(brakePin, OUTPUT);
    timerAlarmWrite(scTimer, ZERO_SPEED_TIMEOUT, false);
    timerAlarmEnable(scTimer);
}

// Configure ESP32 PWM for Motor
void SpeedMotor::setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution) {
    pwmFreq = pwm_Freq;
    resolution = pwm_resolution;
    channel = pwm_channel;
    ledcAttachPin(enable, channel);
    ledcSetup(channel, pwmFreq, resolution);
}

// Read Encoder States and Update Position, Speed and PCR
void SpeedMotor::update() {
    unsigned int t = timerRead(scTimer);
    if (scFlag && !brake) {
        scFlag = 0;
        if (direction) position --;
        else position ++;
        speed = 1e6/t;
        timerWrite(scTimer, 0);
    }
    if (t >= ZERO_SPEED_TIMEOUT) speed = 0;
    // Calculate rate of change of power (PCR) applied for direction switch braking
    if (loopCycles % 50 == 0) {
        powerChangeRate = 1e6*abs((power - prevPower))/(elapsedTime - prevTime);
    }
    if ((abs(power) < abs(powerChangeRate) * BRAKE_TIME_MULTIPLIER) && abs(prevPower) > abs(power)) {
        brake = 1;
    } if (abs(prevPower) < abs(power)) {
        brake = 0;
    }
    if (loopCycles % 50 == 0) prevPower = power;
}

void SpeedMotor::setPower(unsigned int pwr) {
    power = pwr;
}

void SpeedMotor::setDirection(bool dir) {
    direction = dir;
}

unsigned short int SpeedMotor::getPower() {
    return power;
}

volatile bool* SpeedMotor::getInterrupt() {
    return &scFlag;
}

byte SpeedMotor::getSC() {
    return scPin;
}

long SpeedMotor::getPosition() {
    return position;
}

bool SpeedMotor::getDirection() {
    return direction;
}

bool SpeedMotor::getBrake() {
    return brake;
}

int SpeedMotor::getPCR() {
    return powerChangeRate;
}

float SpeedMotor::getSpeed() {
    return speed;
}

void SpeedMotor::resetPosition() {
    position = 0;
}

void SpeedMotor::setBrake(bool brk) {
    brake = brk;
}