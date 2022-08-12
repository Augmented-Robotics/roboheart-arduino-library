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

RoboHeart::RoboHeart() : mpu(MPU6050(Wire)) { _debug = NULL; }

RoboHeart::RoboHeart(Stream& debug)
    : _debug(&debug),
      motor0(RoboHeartDRV8836(debug)),
      motor1(RoboHeartDRV8836(debug)),
      motor2(RoboHeartDRV8836(debug)),
      stepper(RoboHeartStepperMotor(debug)),
      mpu(MPU6050(Wire)) {}

RoboHeart::~RoboHeart(void) {}

bool RoboHeart::begin(bool calc_mpu_offsets) {
    Wire.setPins(I2C_SDA, I2C_SCL);
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

    if (status == 0 && calc_mpu_offsets) {
        DEBUG_LN_IDENTIFIER("Calculating offsets, do not move MPU6050");
        delay(1000);
        mpu.calcOffsets(true, true);  // gyro and accelero
        DEBUG_LN_IDENTIFIER("Done!\n");
    }

    // MOTOR SETUP

    motor0.begin(GPIO_MA_MODE, GPIO_MA_PH_IN1, GPIO_MA_EN_IN2, GPIO_MA_SLEEP);
    motor1.begin(GPIO_MB_MODE, GPIO_MB_PH_IN1, GPIO_MB_EN_IN2, GPIO_MB_SLEEP);
    motor2.begin(GPIO_MC_MODE, GPIO_MC_PH_IN1, GPIO_MC_EN_IN2, GPIO_MC_SLEEP);

    stepper.begin(&motor0, &motor1);

    return true;
}

void RoboHeart::beat() { mpu.update(); }

void RoboHeart::handleMotorMessage(
    Motor_MSG_t motormsg,
    char* response) {  // TODO: response not a good arduino practice
    switch (motormsg.command) {
        case 1:
            // forward
            motor2.forward(motormsg.speed);
            sprintf(response, "forward");
            break;
        case 2:
            // reverse
            motor2.reverse(motormsg.speed);
            sprintf(response, "reverse");
            break;
        case 3:
            // right
            sprintf(response, "right");
            motor1.forward(motormsg.steering_power);
            break;
        case 4:
            // left
            sprintf(response, "left");
            motor1.reverse(motormsg.steering_power);
            break;
        case 5:
            // forward and right
            sprintf(response, "forward and right");
            motor2.forward(motormsg.speed);
            motor1.forward(motormsg.steering_power);
            break;
        case 6:
            // forward and left
            sprintf(response, "forward and left");
            motor2.forward(motormsg.speed);
            motor1.reverse(motormsg.steering_power);
            break;
        case 7:
            // reverse and right
            sprintf(response, "reverse and right");
            motor2.reverse(motormsg.speed);
            motor1.forward(motormsg.steering_power);
            break;
        case 8:
            // reverse and left
            sprintf(response, "reverse and left");
            motor2.reverse(motormsg.speed);
            motor1.reverse(motormsg.steering_power);
            break;
        case 0:
            // STOP
            sprintf(response, "STOP");
            motor2.brake();
            motor1.brake();

            break;
        default:
            // STOP
            sprintf(response, "ERROR: %d", motormsg.command);
            motor2.brake();
            motor1.brake();
            break;
    }
}