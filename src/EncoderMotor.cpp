#include "EncoderMotor.h"


// Configure Pin Modes
void EncoderMotor::configurePins(byte encA_pin, byte encB_pin, byte enable_pin, byte in1_pin, byte in2_pin) {
    encA = encA_pin;
    encB = encB_pin;
    enable = enable_pin;
    in1 = in1_pin;
    in2 = in2_pin;
    pinMode(encA, INPUT);
    pinMode(encB, INPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
}
    
// Configure ESP32 PWM and Dynamic Frequency Bounds
void EncoderMotor::configurePWM(unsigned int pwmFreq_min, unsigned int pwmFreq_max, float power_threshold, byte freq_steps, byte pwm_channel, byte pwm_resolution) {
    pwmFreqMin = pwmFreq_min;
    pwmFreqMax = pwmFreq_max;
    resolution = pwm_resolution;
    freqSteps = freq_steps;
    channel = pwm_channel;
    maxPower = pow(2,resolution) - 1;
    threshold = power_threshold * maxPower;
    powerBuffer = (0.25 * threshold) / freqSteps;
    lowBuffer = -powerBuffer;
    highBuffer = powerBuffer;
    frequency = pwmFreqMax;
    prevFrequency = pwmFreqMax;
    stepHeight = (pwmFreqMax - pwmFreqMin) / freqSteps;
    ledcAttachPin(enable, channel);
    ledcSetup(channel, frequency, resolution);
}

// Update Power-Dependent Dynamic Frequency
void EncoderMotor::updateFreq() {
    if (power < threshold) {
        if (!(power > lowBuffer && power < highBuffer)) {
            frequency = stepFunction(power);
        }
    } else {
        frequency = pwmFreqMax;
    }
    if (prevFrequency != frequency) {
        highBuffer = power + powerBuffer;
        lowBuffer = power - powerBuffer;
        ledcChangeFrequency(channel, frequency, resolution);
    }
    prevFrequency = frequency;
}

// Compute Stepped Linear Frequency Function
unsigned int EncoderMotor::stepFunction(unsigned int x) {
    return (floor(((pwmFreqMax - pwmFreqMin) * x) / (threshold * stepHeight)) * stepHeight) + pwmFreqMin;
}

// Update and Read Encoder States
void EncoderMotor::updatePosition() {
    byte encA_read = digitalRead(encA);
    byte encB_read = digitalRead(encB);
    if (encAFlag == 1) {
        if (encA_read != encB_read) {
            position ++;
        } else {
            position --;
        }
    } else if (encBFlag == 1) {
        if (encA_read != encB_read) {
            position --;
        } else {
            position ++;
        }
    }
}

// Drive Motor using PWM and Direction controls
void EncoderMotor::driveMotor(unsigned int pwr, byte dir) {
    power = pwr;
    direction = dir;
    updateFreq();
    ledcWrite(channel, power);
    if (direction == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in2, HIGH);
        digitalWrite(in1, LOW);
    }
}

void EncoderMotor::driveMotor() {
    updateFreq();
    ledcWrite(channel, power);
    if (direction == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in2, HIGH);
        digitalWrite(in1, LOW);
    }
}

void EncoderMotor::stopMotor() {
    power = 0;
    updateFreq();
    ledcWrite(channel, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void EncoderMotor::setPower(unsigned int pwr) {
    power = pwr;
}

void EncoderMotor::setDirection(byte dir) {
    direction = dir;
}

unsigned int EncoderMotor::getMaxPower() {
    return maxPower;
}
