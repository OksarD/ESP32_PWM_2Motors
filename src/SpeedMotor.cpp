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
    currentTime = esp_timer_get_time();
    powerChangeRate = 1e6*(power - prevPower)/(currentTime - prevTime);
    short brakeThreshold = powerChangeRate * brakeTimeMultiplier;
    if (power < brakeThreshold && power > -brakeThreshold) brake = 1;
    else brake = 0;
    prevTime = currentTime;
    prevPower = power;
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

int SpeedMotor::getPosition() {
    return position;
}

bool SpeedMotor::getDirection() {
    return direction;
}

bool SpeedMotor::getBrake() {
    return brake;
}

float SpeedMotor::getPCR() {
    return powerChangeRate;
}

void SpeedMotor::resetPosition() {
    position = 0;
}