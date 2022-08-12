/*!
 *  @file RoboHeart.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#ifndef RoboHeart_h
#define RoboHeart_h

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>

#include "RoboHeartDRV8836.h"
#include "RoboHeartStepperMotor.h"
#include "pins_RoboHeart.h"

#define MPU6050_I2C_ADDR 0x69 // AD0 HIGH

typedef struct {
    int command;
    int speed;
    int steeringPower;
} MotorMSGType;

class RoboHeart {
   public:
    RoboHeart();
    RoboHeart(Stream& debug);
    ~RoboHeart();

    bool begin(bool mpuOffsetsCalc = true);
    void beat();

    void handleMotorMessage(MotorMSGType motorMSG, char* response);

    RoboHeartStepperMotor stepper;
    RoboHeartDRV8836 motorA;
    RoboHeartDRV8836 motorB;
    RoboHeartDRV8836 motorC;
    MPU6050 mpu;

   private:
    Stream* _debug = NULL;
};

#endif