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
float RoboHeart::_driftX;
float RoboHeart::_rotationY;
float RoboHeart::_driftY;
float RoboHeart::_rotationZ;
float RoboHeart::_driftZ;

void RoboHeart::rotationCallBack(void *pvParameter)
{
    uint32_t start_time_us = (uint32_t)esp_timer_get_time();
    
    float prev_angular_velocityX = 0;
    float prev_angular_velocityY = 0;
    float prev_angular_velocityZ = 0;
    while(true){
        uint32_t actual_time_us = (uint32_t)esp_timer_get_time();
        uint32_t diff_us = actual_time_us - start_time_us; 
        if(diff_us > PERIOD_US){
            float angular_velocityX = imu.readFloatGyroX() - _driftX;
            float angular_velocityY = imu.readFloatGyroY() - _driftY;
            float angular_velocityZ = imu.readFloatGyroZ() - _driftZ;

            _rotationX += ((prev_angular_velocityX+angular_velocityX)/2)*(0.000001*diff_us);
            _rotationY += ((prev_angular_velocityY+angular_velocityY)/2)*(0.000001*diff_us);
            _rotationZ += ((prev_angular_velocityZ+angular_velocityZ)/2)*(0.000001*diff_us);

            prev_angular_velocityX = angular_velocityX;
            prev_angular_velocityY = angular_velocityY;
            prev_angular_velocityZ = angular_velocityZ;
            if (_rotationX > 360) {
            _rotationX -= 360;
            } else if (_rotationX < 0) {
            _rotationX += 360;
            }
            if (_rotationY > 360) {
            _rotationY -= 360;
            } else if (_rotationY < 0) {
            _rotationY += 360;
            }
            if (_rotationZ > 360) {
            _rotationZ -= 360;
            } else if (_rotationZ < 0) {
            _rotationZ += 360;
            }
            start_time_us = actual_time_us;
        }
    }
}

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
    
    Serial.println("Automatic calibration started.");
    calculateDiff();
    int counter = 0;
    while (isCalibrated() == 0) {
        counter++;
        if(counter > 3){
            calculateDiff();
            counter = 0;
            Serial.println("Calibration failed, trying again...");
            Serial.println("Please let RoboHeart in stable position...");
        }
    }  
    Serial.println("RoboHeart calibrated");
    xTaskCreate(&rotationCallBack, "RotationTask", 2048, NULL, 5, NULL);
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


void RoboHeart::calculateDiff(int timeout_ms){
    //Serial.println("Drift");
    _driftX = 0;
    _driftY = 0;
    _driftZ = 0;
    for (int i = 0; i < timeout_ms; i++) {
    _driftX += imu.readFloatGyroX() / timeout_ms;
    _driftY += imu.readFloatGyroY() / timeout_ms;
    _driftZ += imu.readFloatGyroZ() / timeout_ms;
    delay(1);
  }
  Serial.println(_driftX);
  Serial.println(_driftY);
  Serial.println(_driftZ);
}

bool RoboHeart::isCalibrated(int timeout_ms) {
    unsigned long time = millis();
    long counter = 0;
    while ((millis() - time) < timeout_ms) {
        if (abs(imu.readFloatGyroX() - _driftX) > TRESHOLD || abs(imu.readFloatGyroY() - _driftY) > TRESHOLD || abs(imu.readFloatGyroZ() - _driftZ) > TRESHOLD) {
            counter++;
        }
    delay(1);
    }
    //Serial.println(counter);
    return (counter < (timeout_ms / 3));
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