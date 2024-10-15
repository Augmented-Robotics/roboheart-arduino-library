/*!
 *  @file RoboHeart.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeart.h"

#define FILE_IDENTIFIER \
    "ROBOHEART"  // Define identifier before including DebuggerMsgs.h
#include "DebuggerMsgs.h"



LSM6DS3 RoboHeart::imu(I2C_MODE, LSM6DS3_I2C_ADDR);
float RoboHeart::_rotationX;
float RoboHeart::_rotationY;
float RoboHeart::_rotationZ;
tIMUdata RoboHeart::imuData = {0};
EventGroupHandle_t RoboHeart::xIMUeventGroup;
SemaphoreHandle_t RoboHeart::xVarMutex;

RoboHeart::RoboHeart() {}

RoboHeart::RoboHeart(Stream& debug)
    : _debug(&debug),
      motorA(RoboHeartDRV8836(debug)),
      motorB(RoboHeartDRV8836(debug)),
      motorC(RoboHeartDRV8836(debug)),
      stepper(RoboHeartStepperMotor(debug)) {}
    

RoboHeart::~RoboHeart(void) {}

bool RoboHeart::begin() {
    
    Wire.setPins(IMU_SDA, IMU_SCL);
    Wire.begin();
    
    imu.settings.gyroEnabled = 1;
    imu.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
    imu.settings.gyroSampleRate = 104;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
    imu.settings.gyroBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;

    imu.settings.accelEnabled = 1;
    imu.settings.accelODROff = 0;
    imu.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
    imu.settings.accelSampleRate =  imu.settings.gyroSampleRate;
    imu.settings.accelBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;

    imu.settings.tempEnabled = 0;
    
    byte status = imu.begin();

    if (status != 0) {
        // try one more time after a delay
        delay(500);
        status = imu.begin();
    }

    DEBUG_IDENTIFIER("LSM6DS3 status: ");
    DEBUG_LN(status);

    motorA.begin(MOTOR_A_IN1, MOTOR_A_IN2, SLEEP_MOTOR_ABC, MOTOR_A_CHANNEL1,
                 MOTOR_A_CHANNEL2);
    motorB.begin(MOTOR_B_IN1, MOTOR_B_IN2, SLEEP_MOTOR_ABC, MOTOR_B_CHANNEL1,
                 MOTOR_B_CHANNEL2);
    motorC.begin(MOTOR_C_IN1, MOTOR_C_IN2, SLEEP_MOTOR_ABC, MOTOR_C_CHANNEL1,
                 MOTOR_C_CHANNEL2);

    stepper.begin(motorA, motorB);

    setDirectionTurnMotors(motorC, motorB);

    return true;
}


void RoboHeart::setPWM(int motor, int freq, int pwm){
    if(motor == MOTOR_A){
        motorA.configPWM(freq, pwm);
    } else if (motor == MOTOR_B){
        motorB.configPWM(freq, pwm);
    } else if (motor == MOTOR_C){
        motorC.configPWM(freq, pwm);
    }
}

void RoboHeart::setAutomaticRotation(){
    
    xVarMutex = xSemaphoreCreateMutex();
    xIMUeventGroup = xEventGroupCreate();

    xTaskCreatePinnedToCore(IMUcalculateDataTask, "IMUcalculateDataTask", 8192, NULL, configMAX_PRIORITIES - 10, NULL, 0);
    xTaskCreatePinnedToCore(IMUgetDataTask, "IMUgetDataTask", 8192, NULL, configMAX_PRIORITIES - 10, NULL, 0);
}

void RoboHeart::setDirectionTurnMotors(RoboHeartDRV8836& directionMotor,
                                       RoboHeartDRV8836& turnMotor) {
    _directionMotor = &directionMotor;
    _turnMotor = &turnMotor;
}

char* RoboHeart::handleMotorMessage(MotorMSGType motorMSG) {
    char* response = "None";
    RETURN_VAL_WARN_IF_EQUAL(_directionMotor, NULL, response)

    switch (motorMSG.command) {
        case 1:
            _directionMotor->forward(motorMSG.speed);
            response = "forward";
            break;
        case 2:
            _directionMotor->reverse(motorMSG.speed);
            response = "reverse";
            break;
        case 3:
            _turnMotor->forward(motorMSG.steeringPower);
            response = "right";
            break;
        case 4:
            _turnMotor->reverse(motorMSG.steeringPower);
            response = "left";
            break;
        case 5:
            _directionMotor->forward(motorMSG.speed);
            _turnMotor->forward(motorMSG.steeringPower);
            response = "forward and right";
            break;
        case 6:
            _directionMotor->forward(motorMSG.speed);
            _turnMotor->reverse(motorMSG.steeringPower);
            response = "forward and left";
            break;
        case 7:
            _directionMotor->reverse(motorMSG.speed);
            _turnMotor->forward(motorMSG.steeringPower);
            response = "reverse and right";
            break;
        case 8:
            _directionMotor->reverse(motorMSG.speed);
            _turnMotor->reverse(motorMSG.steeringPower);
            response = "reverse and left";
            break;
        case 0:
            _directionMotor->brake();
            _turnMotor->brake();
            response = "STOP";
            break;
        default:
            _directionMotor->brake();
            _turnMotor->brake();
            response = "ERROR";
            break;
    }
    return response;
}

float RoboHeart::getRotationX(){
     return this->_rotationX;
}

float RoboHeart::getRotationY(){
     return this->_rotationY;
}

float RoboHeart::getRotationZ(){
     return this->_rotationZ;
}


void RoboHeart::resetGyro(){
    this->_rotationX = 0;
    this->_rotationY = 0;
    this->_rotationZ = 0;
}

float RoboHeart::getTemperatureC(){
    return (imu.readRawTemp() / 256.) + 25;
}

float RoboHeart::getTemperatureF(){
    return (getTemperatureC()* 1.8) + 32 ;
}

void RoboHeart::computeEulerRates(float omega_x, float omega_y, float omega_z, float phi, float theta, float* dphi, float* dtheta, float* dpsi){
    // Transformation matrix elements
    float cos_phi = cosf(phi);
    float sin_phi = sinf(phi);
    float cos_theta = cosf(theta);

    // Compute the rates of change of Euler angles
    *dphi = omega_x + (omega_y * sin_phi + omega_z * cos_phi) * tanf(theta);
    *dtheta = omega_y * cos_phi - omega_z * sin_phi;
    *dpsi = (omega_y * sin_phi + omega_z * cos_phi) / cos_theta;
}

void RoboHeart::rungeKutta4(float* phi, float* theta, float* psi, float omega_x, float omega_y, float omega_z, float dt){
    // Intermediate variables for Runge-Kutta
    float k1_phi, k2_phi, k3_phi, k4_phi;
    float k1_theta, k2_theta, k3_theta, k4_theta;
    float k1_psi, k2_psi, k3_psi, k4_psi;

    float phi_temp, theta_temp, psi_temp;
    float dphi, dtheta, dpsi;

    // k1 values
    computeEulerRates(omega_x, omega_y, omega_z, *phi, *theta, &dphi, &dtheta, &dpsi);
    k1_phi = dphi * dt;
    k1_theta = dtheta * dt;
    k1_psi = dpsi * dt;

    // k2 values
    phi_temp = *phi + k1_phi / 2;
    theta_temp = *theta + k1_theta / 2;
    psi_temp = *psi + k1_psi / 2;
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k2_phi = dphi * dt;
    k2_theta = dtheta * dt;
    k2_psi = dpsi * dt;

    // k3 values
    phi_temp = *phi + k2_phi / 2;
    theta_temp = *theta + k2_theta / 2;
    psi_temp = *psi + k2_psi / 2;
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k3_phi = dphi * dt;
    k3_theta = dtheta * dt;
    k3_psi = dpsi * dt;

    // k4 values
    phi_temp = *phi + k3_phi;
    theta_temp = *theta + k3_theta;
    psi_temp = *psi + k3_psi;
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k4_phi = dphi * dt;
    k4_theta = dtheta * dt;
    k4_psi = dpsi * dt;

    // Update the Euler angles
    *phi += (k1_phi + 2 * k2_phi + 2 * k3_phi + k4_phi) / 6.0f;
    *theta += (k1_theta + 2 * k2_theta + 2 * k3_theta + k4_theta) / 6.0f;
    *psi += (k1_psi + 2 * k2_psi + 2 * k3_psi + k4_psi) / 6.0f;
}

void RoboHeart::rungeKutta2(float* phi, float* theta, float* psi, float omega_x, float omega_y, float omega_z, float dt){
    // Intermediate variables for Runge-Kutta
    float k1_phi, k2_phi;
    float k1_theta, k2_theta;
    float k1_psi, k2_psi;

    float phi_temp, theta_temp, psi_temp;
    float dphi, dtheta, dpsi;

    // k1 values (slopes at the beginning of the interval)
    computeEulerRates(omega_x, omega_y, omega_z, *phi, *theta, &dphi, &dtheta, &dpsi);
    k1_phi = dphi * dt;
    k1_theta = dtheta * dt;
    k1_psi = dpsi * dt;

    // Intermediate angles using k1 (midpoint approximation)
    phi_temp = *phi + k1_phi / 2;
    theta_temp = *theta + k1_theta / 2;
    psi_temp = *psi + k1_psi / 2;

    // k2 values (slopes at the midpoint)
    computeEulerRates(omega_x, omega_y, omega_z, phi_temp, theta_temp, &dphi, &dtheta, &dpsi);
    k2_phi = dphi * dt;
    k2_theta = dtheta * dt;
    k2_psi = dpsi * dt;

    // Update the Euler angles using the second-order RK method
    *phi += k2_phi;
    *theta += k2_theta;
    *psi += k2_psi;
}

void RoboHeart::IMUgetDataTask(void *pvParameter){
    uint8_t status = 0;
  tIMUdata data = {0};

  while(true) {
    imu.readRegister(&status, LSM6DS3_ACC_GYRO_STATUS_REG);
    if (status & LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL) {
      imu.readRegisterRegion((uint8_t *) &data, LSM6DS3_ACC_GYRO_OUTX_L_G, 12);
      if (xSemaphoreTake(xVarMutex, portMAX_DELAY)) {
        imuData = data;
        xSemaphoreGive(xVarMutex);
      }
      xEventGroupSetBits(xIMUeventGroup, NEW_IMU_DATA_BIT);
    }
    vTaskDelay(((1000/imu.settings.gyroSampleRate)-1)/portTICK_PERIOD_MS);
  }
}
void RoboHeart::IMUcalculateDataTask(void *pvParameter){
    int64_t offsetRX = 0.0;
  int64_t offsetRY = 0.0;
  int64_t offsetRZ = 0.0;

  int16_t previousAX = 0;
  int16_t previousAY = 0;
  int16_t previousAZ = 0;

  int64_t sumRX = 0;
  int64_t sumRY = 0;
  int64_t sumRZ = 0;

  float dphi, dtheta, dpsi;

  bool calibration = false;
  int64_t calibrationSteps = 0;
  uint8_t calibrationCountdown = CALIBRATION_COUNTDOWN;

  tIMUdata inputData = {0};
  float phi = 0.0, theta = 0.0, psi = 0.0;

  while(true) {
    xEventGroupWaitBits(xIMUeventGroup, NEW_IMU_DATA_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    if (xSemaphoreTake(xVarMutex, portMAX_DELAY)) {
      inputData = imuData;
      xSemaphoreGive(xVarMutex);
    }

    if ((abs(previousAX - inputData.aX) < ACCELEROMETER_CUTOFF) && (abs(previousAY - inputData.aY) < ACCELEROMETER_CUTOFF) && (abs(previousAZ - inputData.aZ) < ACCELEROMETER_CUTOFF)) {
      previousAX = inputData.aX;
      previousAY = inputData.aY;
      previousAZ = inputData.aZ;

      if (calibration) {
        sumRX += inputData.rX;
        sumRY += inputData.rY;
        sumRZ += inputData.rZ;
        calibrationSteps++;
      } else {
        if (calibrationCountdown) {
          calibrationCountdown--;
          continue;
        }

        calibrationCountdown = CALIBRATION_COUNTDOWN;
        calibration = true;
        sumRX = inputData.rX;
        sumRY = inputData.rY;
        sumRZ = inputData.rZ;
        calibrationSteps = 1;
      }
      continue;
    } else {
      calibrationCountdown = CALIBRATION_COUNTDOWN;
      previousAX = inputData.aX;
      previousAY = inputData.aY;
      previousAZ = inputData.aZ;

      if (calibration) {
        calibration = false;
        offsetRX = sumRX / calibrationSteps;
        offsetRY = sumRY / calibrationSteps;
        offsetRZ = sumRZ / calibrationSteps;
      }
    }

    float omega_x =  imu.calcGyro(inputData.rX - offsetRX) * DEG_TO_RAD;
    float omega_y =  imu.calcGyro(inputData.rY - offsetRY) * DEG_TO_RAD;
    float omega_z =  imu.calcGyro(inputData.rZ - offsetRZ) * DEG_TO_RAD;

    //rungeKutta2(&phi, &theta, &psi, omega_x, omega_y, omega_z, 1.0 / imu.settings.gyroSampleRate);
    rungeKutta4(&phi, &theta, &psi, omega_x, omega_y, omega_z, 1.0 /  imu.settings.gyroSampleRate);

    if (phi >= TWO_PI) {
      phi -= TWO_PI;
    } else if (phi < 0.0) {
      phi += TWO_PI;
    }
    _rotationX = phi * RAD_TO_DEG;
    if (theta >= TWO_PI) {
      theta -= TWO_PI;
    } else if (theta < 0.0) {
      theta += TWO_PI;
    }
    _rotationY = theta * RAD_TO_DEG;
    if (psi >= TWO_PI) {
      psi -= TWO_PI;
    } else if (psi < 0.0) {
      psi += TWO_PI;
    }
    _rotationZ = psi * RAD_TO_DEG;
  }
}