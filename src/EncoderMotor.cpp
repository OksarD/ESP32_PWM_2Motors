#include "EncoderMotor.h"

// Configure Pins for Motor
void EncoderMotor::setupPins(byte encA_pin, byte encB_pin, byte enable_pin, byte dir_pin, byte brake_pin) {
    encA = encA_pin;
    encB = encB_pin;
    enable = enable_pin;
    dirPin = dir_pin;
    brakePin = brake_pin;
    pinMode(encA, INPUT);
    pinMode(encB, INPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(brakePin, OUTPUT);
}
    
// Configure ESP32 PWM for Motor
void EncoderMotor::setupPWM(unsigned int pwm_Freq, byte pwm_channel, byte pwm_resolution) {
    pwmFreq = pwm_Freq;
    resolution = pwm_resolution;
    channel = pwm_channel;
    maxPower = pow(2,resolution) - 1;
    ledcAttachPin(enable, channel);
    ledcSetup(channel, pwmFreq, resolution);
}

// Update and Read Encoder States
void EncoderMotor::updatePosition() {
    bool encA_read = digitalRead(encA);
    bool encB_read = digitalRead(encB);
    if (encAFlag) {
        if (encB_read != encA_read) position ++;
        else position --;
        encAFlag = 0;
    } else if (encBFlag) {
        if (encA_read != encB_read) position --;
        else position ++;
        encBFlag = 0;
    }
}

// Drive Motor using PWM and Direction controls
void EncoderMotor::driveMotor(unsigned int pwr, bool dir) {
    power = pwr;
    direction = dir;
    ledcWrite(channel, power);
    if (direction) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
}

void EncoderMotor::driveMotor() {
    ledcWrite(channel, power);
    if (direction) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
}

void EncoderMotor::stopMotor() {
    power = 0;
    ledcWrite(channel, power);
}

void EncoderMotor::brakeMotor() {
    power = 0;
    ledcWrite(channel, power);
    digitalWrite(brakePin, HIGH);
}

void EncoderMotor::setPower(unsigned int pwr) {
    power = pwr;
}

void EncoderMotor::setDirection(bool dir) {
    direction = dir;
}

unsigned short int EncoderMotor::getMaxPower() {
    return maxPower;
}

unsigned short int EncoderMotor::getPower() {
    return power;
}

volatile bool* EncoderMotor::getInterruptA() {
    return &encAFlag;
}

volatile bool* EncoderMotor::getInterruptA() {
    return &encBFlag;
}