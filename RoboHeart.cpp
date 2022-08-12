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

    motorA.begin(MOTOR_A_IN1, MOTOR_A_IN2, SLEEP_MOTOR_ABC);
    motorB.begin(MOTOR_B_IN1, MOTOR_B_IN2, SLEEP_MOTOR_ABC);
    motorC.begin(MOTOR_C_IN1, MOTOR_C_IN2, SLEEP_MOTOR_ABC);

    stepper.begin(&motorA, &motorB);

    return true;
}

void RoboHeart::beat() { mpu.update(); }

void RoboHeart::handleMotorMessage(
    MotorMSGType motorMSG,
    char* response) {  // TODO: response not a good arduino practice
    switch (motorMSG.command) {
        case 1:
            // forward
            motorC.forward(motorMSG.speed);
            sprintf(response, "forward");
            break;
        case 2:
            // reverse
            motorC.reverse(motorMSG.speed);
            sprintf(response, "reverse");
            break;
        case 3:
            // right
            sprintf(response, "right");
            motorB.forward(motorMSG.steeringPower);
            break;
        case 4:
            // left
            sprintf(response, "left");
            motorB.reverse(motorMSG.steeringPower);
            break;
        case 5:
            // forward and right
            sprintf(response, "forward and right");
            motorC.forward(motorMSG.speed);
            motorB.forward(motorMSG.steeringPower);
            break;
        case 6:
            // forward and left
            sprintf(response, "forward and left");
            motorC.forward(motorMSG.speed);
            motorB.reverse(motorMSG.steeringPower);
            break;
        case 7:
            // reverse and right
            sprintf(response, "reverse and right");
            motorC.reverse(motorMSG.speed);
            motorB.forward(motorMSG.steeringPower);
            break;
        case 8:
            // reverse and left
            sprintf(response, "reverse and left");
            motorC.reverse(motorMSG.speed);
            motorB.reverse(motorMSG.steeringPower);
            break;
        case 0:
            // STOP
            sprintf(response, "STOP");
            motorC.brake();
            motorB.brake();

            break;
        default:
            // STOP
            sprintf(response, "ERROR: %d", motorMSG.command);
            motorC.brake();
            motorB.brake();
            break;
    }
}