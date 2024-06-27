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

// arduino-esp32 starts assigning channel from the top (channel 7)
// lower the chances of collision by starting from the bottom
// TODO: Remove on the new arduino-esp32 release
#define MOTOR_A_CHANNEL1 0
#define MOTOR_A_CHANNEL2 1
#define MOTOR_B_CHANNEL1 2
#define MOTOR_B_CHANNEL2 3
#define MOTOR_C_CHANNEL1 4
#define MOTOR_C_CHANNEL2 5

LSM6DS3 RoboHeart::imu;
float RoboHeart::_rotationX;
float RoboHeart::_driftX;
float RoboHeart::_rotationY;
float RoboHeart::_driftY;
float RoboHeart::_rotationZ;
float RoboHeart::_driftZ;

void RoboHeart::rotationCallBack()
{
    _rotationX += (imu.readFloatGyroX() - _driftX)*0.01;
    //_rotationY += (imu.readFloatGyroY() - _driftY)*0.005;
    //_rotationZ += (imu.readFloatGyroZ() - _driftZ)*0.005;
    if (_rotationX > 360) {
      _rotationX -= 360;
    } else if (_rotationX < 0) {
      _rotationX += 360;
    }
    /*if (_rotationY > 360) {
      _rotationY -= 360;
    } else if (_rotationY < 0) {
      _rotationY += 360;
    }
    if (_rotationZ > 360) {
      _rotationZ -= 360;
    } else if (_rotationZ < 0) {
      _rotationZ += 360;
    }*/
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

    DEBUG_IDENTIFIER("MPU6050 status: ");
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

void RoboHeart::setAutomaticRotation(){
    calculateDiff();
    while (isCalibrated() == 0) {}  
    
    PeriodicTimer t = PeriodicTimer(rotationCallBack, 10000);
    t.start();
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
    Serial.println("Drift");
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
    
    Serial.println(counter);
    return (counter < (timeout_ms / 5));
}