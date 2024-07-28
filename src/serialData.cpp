#include "serialData.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL

#define INTERRUPT_PIN_I2C 23 

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
short rcAnalogs[4]; 
bool ch3State = 0;
bool ch4State = 0;
byte ch7State = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t rcBuffer[9];   // RC storage buffer
bool I2CdataFlag = 0;

void IRAM_ATTR IMU_ISR() {
    I2CdataFlag = 1;
}

void IMUinit() {
    mpu.initialize();
    pinMode(INTERRUPT_PIN_I2C, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_I2C), IMU_ISR, RISING);
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(83);
    mpu.setYGyroOffset(-71);
    mpu.setZGyroOffset(-5);
    mpu.setZAccelOffset(1426); // 1688 factory default for my test chip
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        // mpu.CalibrateAccel(6);
        // mpu.CalibrateGyro(6);
        // mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        delay(1000);
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void readIMUdata() {
        // if programming failed, don't try to do anything
        if (!dmpReady) return;
        // read a packet from FIFO
        if (I2CdataFlag) {
            I2CdataFlag = 0;
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            #endif
            #ifdef OUTPUT_READABLE_REALACCEL
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            #endif
            Wire.flush();
        }
        } else {
            //Serial.print("No IMU data!");
        }
}

void readRCdata() {
    Wire.flush();
    Wire.requestFrom(0x60, 9);
    if(Wire.available()) {
            Wire.readBytes(rcBuffer, 9);
        for (byte i = 0; i < 4; i++) {
            rcAnalogs[i] = (rcBuffer[(2*i)+1] << 8) + rcBuffer[2*i];
        }
        ch3State = rcBuffer[8] & 0b0001;
        ch4State = (rcBuffer[8] & 0b0010) >> 1;
        ch7State = rcBuffer[8] >> 2;
    }
    Wire.flush();
}

void readMotorSpeed(encoderMotor motor_object) {
    odrive_serial << "r axis" << motor_object.axis << ".encoder.vel_estimate\n";
    motor_object.speed = odrive.readFloat();
}

void writeMotorOutputsTask(void *paramaters) {
    for(;;) {
        char c = ' ';
        if (Serial.available()) c = Serial.read();
        // Run when r pressed or if running paramater already set
        if (c == 'r' || running) {
            if(odrive_serial.availableForWrite()) {
                odrive.SetCurrent(m2.axis, -m2output* rampLevel);
                odrive.SetCurrent(m1.axis, m1output* rampLevel);
            }
            running = 1;
        }
        // Sinusoidal test move (only while in velocity control)
        if (c == 't') {
            Serial.println("Executing test move");
            for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
            float pos_m0 = -ODRIVE_MAX_VELOCITY * sin(ph);
            float pos_m1 = ODRIVE_MAX_VELOCITY * sin(ph);
            odrive.SetVelocity(0, pos_m0);
            odrive.SetVelocity(1, pos_m1);
            delay(10);
            }
        }
        // stop odrive commmunication
        if (c == 's') {
            running = 0;
            Serial.println("Stopped.");
        }
        vTaskSuspend(NULL);
    }
}