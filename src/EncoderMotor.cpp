#include "EncoderMotor.h"

// Configure Pin Modes
void EncoderMotor::setupPins(byte encA_pin, byte encB_pin, byte enable_pin, byte in1_pin, byte in2_pin) {
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
void EncoderMotor::setupPWM(unsigned int pwmFreq_min, unsigned int pwmFreq_max, byte pwm_channel, byte pwm_resolution) {
    pwmFreqMin = pwmFreq_min;
    pwmFreqMax = pwmFreq_max;
    resolution = pwm_resolution;
    channel = pwm_channel;
    maxPower = pow(2,resolution) - 1;
    freqSteps = 5;
    threshold = 0.5 * maxPower;
    tolerance = (0.25 * threshold) / freqSteps;
    lowToleranceEdge = 0;
    highToleranceEdge = 0;
    frequency = pwmFreqMax;
    prevFrequency = pwmFreqMax;
    stepHeight = (pwmFreqMax - pwmFreqMin) / freqSteps;
    ledcAttachPin(enable, channel);
    ledcSetup(channel, frequency, resolution);
}

// Adjust Dynamic Frequency Paramaters
void EncoderMotor::changeFreqParamaters(float max_power_threshold = -1, byte frequency_steps = -1) {
    if (frequency_steps != -1) { frequency = frequency_steps; }
    if (max_power_threshold != -1) { maxPower = max_power_threshold * maxPower; }
    //Update dependent variables
    tolerance = (0.25 * threshold) / freqSteps;
    stepHeight = (pwmFreqMax - pwmFreqMin) / freqSteps;    
}

// Update Power-Dependent Dynamic Frequency
void EncoderMotor::updateFreq() {
    if (power < threshold) {
        if (!(power > lowToleranceEdge && power < highToleranceEdge)) {
            frequency = stepFunction(power);
        }
    } else {
        frequency = pwmFreqMax;
    }
    // Generate tolerance interval when freuency steps, to prevent quick 
    // frequency changes due to oscillating power values
    if (prevFrequency != frequency) {
        highToleranceEdge = power + tolerance;
        lowToleranceEdge = power - tolerance;
        ledcChangeFrequency(channel, frequency, resolution);
    }
    prevFrequency = frequency;
}

// Compute Stepped Linear Frequency Function
unsigned short int EncoderMotor::stepFunction(unsigned int x) {
    return (floor(((pwmFreqMax - pwmFreqMin) * x) / (threshold * stepHeight)) * stepHeight) + pwmFreqMin;
}

// Update and Read Encoder States
void EncoderMotor::updatePosition() {
    byte encA_read = digitalRead(encA);
    byte encB_read = digitalRead(encB);
    if (encAFlag == 1) {
        if (encB_read != encA_read) {
            position ++;
        } else {
            position --;
        }
        encAFlag = 0;
    
    } else if (encBFlag == 1) {
        if (encA_read != encB_read) {
            position --;
        } else {
            position ++;
        }
        encBFlag = 0;
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

unsigned short int EncoderMotor::getMaxPower() {
    return maxPower;
}
