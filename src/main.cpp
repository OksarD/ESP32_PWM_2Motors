#include <Project.h>
#include <serialData.h>
#include <Control.h>

// Prototypes
void printData();

// Task handles
TaskHandle_t BalanceTask_handle = NULL;
TaskHandle_t SteeringTask_handle = NULL;
TaskHandle_t writeMotorOutputsTask_handle = NULL;

void BalanceTask(void *paramaters) {
  for(;;) {
    readIMUdata();
    targetAngle = calibrationOffset + throttle; // requires RC data
    angle = ypr[2]*180/M_PI;
    BalanceController();
    vTaskSuspend(NULL);
  }
}

void SteeringTask(void *paramaters) {
  for(;;) {
    readMotorSpeed(m1);
    readMotorSpeed(m1);
    SteeringController();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    vTaskSuspend(NULL);
  }
}

// Main Setup
void setup()
{
  Wire.begin();
  Wire.setClock(200000); // 100kHz I2C clock.
  Serial.begin(115200);
  Wire.setTimeOut(100);
  odrive_serial.begin(115200, SERIAL_8N1, ODRIVE_UART_RX, ODRIVE_UART_TX);

  // I2C Setup
  IMUinit();
  xTaskCreatePinnedToCore(BalanceTask, "BalanceTask", 10000, NULL, 1, &BalanceTask_handle, 0);
  xTaskCreatePinnedToCore(SteeringTask, "SteeringTask", 1000, NULL, 1, &SteeringTask_handle, 1);
  xTaskCreatePinnedToCore(writeMotorOutputsTask, "writeMotorOutputsTask", 1000, NULL, 1, &writeMotorOutputsTask_handle, 1);
  Serial.println(F("Initializing I2C devices..."));
  while (!Serial);
  Serial.println("Serial Ready...");
  while(!odrive_serial);
  Serial.println("Serial1 Ready...");

  //ODrive serial and paramater setup
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 50.0f << '\n';
  }

  pinMode(LED_PIN, OUTPUT);
}

// Main Loop
void loop() {

  readRCdata();
  
  // Tuning modes
  #if defined(PROP_TUNING)
    Kp = rcAnalogs[2] * KP_MAX / MAX_POWER;  
  #elif defined(INT_TUNING) 
    Ki = rcAnalogs[2] * KI_MAX / MAX_POWER;
  #elif defined(DERIV_TUNING)
    Kd = rcAnalogs[2] * KD_MAX / MAX_POWER;
  #elif defined(POS_PROP_TUNING)
    posKp = rcAnalogs[2] * POS_KP_MAX / MAX_POWER;
  #elif defined(THROTTLE_TUNING)
    throttleGain = rcAnalogs[2] * THROTTLE_GAIN_MAX / MAX_POWER;
  #elif defined(STEER_PROP_TUNING)
    steerKp = rcAnalogs[2] * STEER_KP_MAX / MAX_POWER;
  #elif defined(STEER_INT_TUNING)
    steerKi = rcAnalogs[2] * STEER_INT_MAX / MAX_POWER;
  #endif

  // Throttle Control
  throttle = rcAnalogs[1] * throttleGain;
  steering = rcAnalogs[0] * 1*ODRIVE_MAX_VELOCITY / MAX_POWER;

  // switch CH7 left for angle tuning, left for motor calibration, middle for operation
  if (ch7State == 0) {
    calibrationOffset = (rcAnalogs[3] - 512) * ANGLE_MAX / MAX_POWER;
    throttle = 0;
    steering = 0;
  } 
  #if defined(ENABLE_MOTOR_CALIBRATION) 
  if (ch7State == 2) {
    if (!motor_calib_state) {
      motor_calib_state = 1;
      for (int num = 0; num < 2; num++) {
        Serial.printf("Calibrating motor %i...\n", num);
        odrive.run_state(num, ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION, false); // don't wait
        delay(5000);
        odrive.run_state(num, ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION, false); // don't wait
        delay(5000);
      }
      delay(22000);
      Serial.println("Done motor calibration.");
      odrive.run_state(0, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false); // don't wait
      odrive.run_state(1, ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL, false); // don't wait
    }
  } else if {
    motor_calib_state = 0
  }
  #endif

  // Update running time
  elapsedTime = esp_timer_get_time();
  
  // Get angle and run balance controller on core 0
  // Get get motor speed and run steering controller on core 1
  if (BalanceTask_handle != NULL) vTaskResume(BalanceTask_handle);
  eTaskState writeState = eTaskGetState(writeMotorOutputsTask_handle);
  if (SteeringTask_handle != NULL && writeState != eRunning) vTaskResume(SteeringTask_handle);

  m1.power = controlOutput + steerOutput;
  m2.power = controlOutput - steerOutput;

  // Kill Switch and Power Ramp Up Logic
  if(ch3State) {
    m1.power = 0;
    m2.power = 0;
    killSwitchReleased = 0;
    rampEnabled = 1;
    rampLevel = 0;
  } else {
    if (!killSwitchReleased) {
      rampTime = elapsedTime;
      killSwitchReleased = 1;
      integral = 0;
      m1.position = 0;
      m2.position = 0;
    }
    if (rampEnabled) {
      if(elapsedTime - rampTime >= RAMP_TIME) {
        rampLevel = 1;
      }
      else rampLevel = (elapsedTime - rampTime)/ RAMP_TIME;
    }
  }

  // Compensate for minimum power and scale
  if (m1.power >= 0) m1output = (m1.power*ODRIVE_MAX_TORQUE/MAX_POWER) + MIN_POWER_1; // scalar is 0.01262
  else m1output = float(m1.power*ODRIVE_MAX_TORQUE/MAX_POWER) - MIN_POWER_1;
  if (m2.power >= 0) m2output = float(m2.power*ODRIVE_MAX_TORQUE/MAX_POWER) + MIN_POWER_2;
  else m2output = float(m2.power*ODRIVE_MAX_TORQUE/MAX_POWER) - MIN_POWER_2;
  
  // Write to odrive
  if (writeMotorOutputsTask_handle != NULL) vTaskResume(writeMotorOutputsTask_handle);

  // print data every so often
  loopCycles++;
  if (running) printData();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}

void printData()
{
  if (loopCycles % 100 == 0)
  {
    // Main Loop Cycles
    // Serial.printf("c: %i, ", loopCycles);

    // Motor Data
    Serial.printf("Pwr1: %.2f, Out1: %.2f ", m1.power, m1output*rampLevel);
    Serial.printf("Spd1: %.2f, ", m1.speed);
    //Serial.printf("Pwr2: %.2f, Out2: %.2f ", m2.power, m2output*rampLevel);
    // Serial.printf("Spd2: %.2f, ", m2.speed);
    Serial.printf("Diff: %.2f, ", speedDiff);
    // IMU Data
    //Serial.printf("roll: %.4f, trim: %.2f" ,ypr[2]*180/M_PI, calibrationOffset);
    
    // PID Data
    //Serial.printf("gain: %.4f, %.4f, %.4f, ", Kp, Ki, Kd);
    //Serial.printf("PID: %.4f, %.4f, %.4f, ", proportional, integral, derivative);
    Serial.printf("StrGain: %.2f, %.2f, ", steerKp, steerKi);
    Serial.printf("StrPI: %.2f, %.2f, ", steerProportional, steerIntegral);
    Serial.printf("StrErr: %.2f, ", steeringError);
    // RC Data
    // Serial.printf("rc: %i, %i, %i, %i, %i, %i, %i, ", rcAnalogs[0], rcAnalogs[1], rcAnalogs[2], rcAnalogs[3], ch3State, ch4State, ch7State);
    
    // Other Data
    // Serial.printf("targ: %.4f, ", targetAngle);
    // Serial.printf("minPwr: %i, ", m1.getPower());
    // Serial.printf("avPos: %i, posKp: %.4f, ", averagePos, posKp);
    //Serial.printf("thrGain: %.6f, strGain: %.6f, ", throttleGain, steeringGain);
    Serial.printf("thr: %.4f, str: %.4f, ", throttle, steering);
    //Serial.printf("ramp: %.4f", rampLevel) ;
  
    Serial.println();
  }
}