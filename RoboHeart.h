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

    void motor0_coast();
    void motor0_sleep(bool sleep = true);
    void motor0_reverse(int speed);
    void motor0_forward(int speed);
    void motor0_brake();

    void motor1_coast();
    void motor1_sleep(bool sleep = true);
    void motor1_reverse(int speed);
    void motor1_forward(int speed);
    void motor1_brake();

    void motor2_coast();
    void motor2_sleep(bool sleep = true);
    void motor2_reverse(int speed);
    void motor2_forward(int speed);
    void motor2_brake();

    void handleMotorMessage(Motor_MSG_t motormsg, char* response);

    float getTemp();
    float getAccX();
    float getAccY();
    float getAccZ();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccAngleX();
    float getAccAngleY();
    float getAngleX();
    float getAngleY();
    float getAngleZ();

    float getGyroXoffset();
    float getGyroYoffset();
    float getGyroZoffset();

    float getAccXoffset();
    float getAccYoffset();
    float getAccZoffset();

    void setGyroOffsets(float x, float y, float z);
    void setAccOffsets(float x, float y, float z);

    RoboHeartStepperMotor stepper;
    RoboHeartDRV8836 motor0;
    RoboHeartDRV8836 motor1;
    RoboHeartDRV8836 motor2;

   private:
    Stream* _debug;
};

extern MPU6050 mpu;

#endif