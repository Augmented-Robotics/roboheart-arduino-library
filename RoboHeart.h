/*!
 *  @file RoboHeart.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#ifndef _ROBOHEART_H
#define _ROBOHEART_H

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>

#include "RoboHeartDRV8836.h"
#include "RoboHeartStepperMotor.h"
#include "pins.h"

typedef struct {
    int command;
    int speed;
    int steering_power;
} Motor_MSG_t;

class RoboHeart {
   public:
    RoboHeart();
    RoboHeart(Stream& debug);
    ~RoboHeart();

    bool begin(bool calc_mpu_offsets = true);
    void beat();

    void handleMotorMessage(Motor_MSG_t motormsg, char* response);

    RoboHeartStepperMotor stepper;
    RoboHeartDRV8836 motor0;
    RoboHeartDRV8836 motor1;
    RoboHeartDRV8836 motor2;
    MPU6050 mpu;

   private:
    Stream* _debug;
};

#endif