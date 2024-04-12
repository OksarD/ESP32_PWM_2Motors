#include "SpeedMotor.h"


// Configure Pins for Motor
void SpeedMotor::setupPins(byte sc_pin, byte pwr_pin, byte dir_pin, byte brake_pin) {
    scPin = sc_pin;
    enable = pwr_pin;
    dirPin = dir_pin;
    brakePin = brake_pin;
    pinMode(scPin, INPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(brakePin, OUTPUT);
}
    
// Configure ESP32 PWM for Motor
void SpeedMotor::setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution) {
    pwmFreq = pwm_Freq;
    resolution = pwm_resolution;
    channel = pwm_channel;
    maxPower = pow(2,resolution) - 1;
    ledcAttachPin(enable, channel);
    ledcSetup(channel, pwmFreq, resolution);
}

// Update and Read Encoder States
void SpeedMotor::update() {
    if (scFlag) {
        scFlag = 0;
        if (direction) {
            position --;
        } else {
            position ++;
        }
    }
}

// Drive Motor using PWM and Direction controls
void SpeedMotor::driveMotor(unsigned int pwr, bool dir) {
    power = pwr;
    targetDirection = dir;
    ledcWrite(channel, power);
    if (targetDirection) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
}

void SpeedMotor::driveMotor() {
    ledcWrite(channel, power);
    if (targetDirection) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
}

void SpeedMotor::stopMotor() {
    power = 0;
    ledcWrite(channel, power);
}

void SpeedMotor::brakeMotor() {
    power = 0;
    ledcWrite(channel, power);
    digitalWrite(brakePin, HIGH);
}

void SpeedMotor::setPower(unsigned int pwr) {
    power = pwr;
}

void SpeedMotor::setDirection(bool dir) {
    targetDirection = dir;
}

unsigned short int SpeedMotor::getMaxPower() {
    return maxPower;
}

unsigned short int SpeedMotor::getPower() {
    return power;
}

volatile bool* SpeedMotor::getInterrupt() {
    return &scFlag;
}

void SpeedMotor::boolInvert(bool var) {
    if (var) {
        var = 0;
    } else {
        var = 1;
    }
}