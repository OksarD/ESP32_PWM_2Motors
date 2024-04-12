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
void EncoderMotor::update() {
    if (encAFlag) {
        boolInvert(encA_state);
        bool encB_state = digitalRead(encB);
        encAFlag = 0;
        if (encB_state != encA_state) {
            direction = 0;
            position ++;
        } else {
            direction = 1;
            position --;
        }
    } else if (encBFlag) {
        boolInvert(encB_state);
        bool encA_state = digitalRead(encA);
        encBFlag = 0;
        if (encA_state != encB_state) {
            direction = 1;
            position --;
        } else {
            direction = 0;
            position ++;
        }
    }
}

// Drive Motor using PWM and Direction controls
void EncoderMotor::driveMotor(unsigned int pwr, bool dir) {
    power = pwr;
    targetDirection = dir;
    ledcWrite(channel, power);
    if (targetDirection) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
}

void EncoderMotor::driveMotor() {
    ledcWrite(channel, power);
    if (targetDirection) {
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
    targetDirection = dir;
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

volatile bool* EncoderMotor::getInterruptB() {
    return &encBFlag;
}

void EncoderMotor::boolInvert(bool var) {
    if (var) {
        var = 0;
    } else {
        var = 1;
    }
}