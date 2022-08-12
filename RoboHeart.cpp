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

RoboHeart::RoboHeart() : mpu(MPU6050(Wire)) {}

RoboHeart::RoboHeart(Stream& debug)
    : _debug(&debug),
      motorA(RoboHeartDRV8836(debug)),
      motorB(RoboHeartDRV8836(debug)),
      motorC(RoboHeartDRV8836(debug)),
      stepper(RoboHeartStepperMotor(debug)),
      mpu(MPU6050(Wire)) {}

RoboHeart::~RoboHeart(void) {}

bool RoboHeart::begin(bool mpuOffsetsCalc) {
    Wire.setPins(IMU_SDA, IMU_SCL);
    Wire.begin();

    mpu.setAddress(MPU6050_I2C_ADDR);
    byte status = mpu.begin();

    if (status != 0) {
        // try one more time after a delay (MPU takes time to power-up)
        delay(500);
        status = mpu.begin();
    }

    DEBUG_IDENTIFIER("MPU6050 status: ");
    DEBUG_LN(status);

    if (status == 0 && mpuOffsetsCalc) {
        DEBUG_LN_IDENTIFIER("Calculating offsets, do not move MPU6050");
        delay(1000);
        mpu.calcOffsets(true, true);  // gyro and accelero
        DEBUG_LN_IDENTIFIER("Done!\n");
    }

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

void RoboHeart::beat() { mpu.update(); }

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