/*!
 *  @file RoboHeart.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#ifndef RoboHeart_h
#define RoboHeart_h

#include <Arduino.h>
#include <SparkFunLSM6DS3.h>
#include <Wire.h>

#include "RoboHeartDRV8836.h"
#include "RoboHeartStepperMotor.h"
#include "pins_RoboHeart.h"

#define LSM6DS3_I2C_ADDR 0x6B // or 0x6A - Address is defined in Library

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
   
    bool begin();

    char* handleMotorMessage(MotorMSGType motorMSG);
    void setDirectionTurnMotors(RoboHeartDRV8836& directionMotor, RoboHeartDRV8836& turnMotor);

    RoboHeartStepperMotor stepper;
    RoboHeartDRV8836 motorA;
    RoboHeartDRV8836 motorB;
    RoboHeartDRV8836 motorC;
    LSM6DS3 imu;
   

   private:
    Stream* _debug = NULL;
    RoboHeartDRV8836* _turnMotor = NULL;
    RoboHeartDRV8836* _directionMotor = NULL;
};

#endif