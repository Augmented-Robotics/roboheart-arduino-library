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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "RoboHeartDRV8836.h"
#include "RoboHeartStepperMotor.h"
#include "pins_RoboHeart.h"
#include <math.h>
#define TRESHOLD 0.1  //treshold in degrees/s

#define MOTOR_A_CHANNEL1 0
#define MOTOR_A_CHANNEL2 1
#define MOTOR_B_CHANNEL1 2
#define MOTOR_B_CHANNEL2 3
#define MOTOR_C_CHANNEL1 4
#define MOTOR_C_CHANNEL2 5

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
    void setAutomaticRotation();

    char* handleMotorMessage(MotorMSGType motorMSG);
    void setDirectionTurnMotors(RoboHeartDRV8836& directionMotor, RoboHeartDRV8836& turnMotor);
    float getRotationX();
    float getRotationY();
    float getRotationZ();

    RoboHeartStepperMotor stepper;
    RoboHeartDRV8836 motorA;
    RoboHeartDRV8836 motorB;
    RoboHeartDRV8836 motorC;
    static LSM6DS3 imu;
   
    static float _rotationX;
    static float _driftX;
    static float _rotationY;
    static float _driftY;
    static float _rotationZ;
    static float _driftZ;
    static bool tick;
    static void rotationCallBack(void *pvParameter);

   private:
    Stream* _debug = NULL;
    RoboHeartDRV8836* _turnMotor = NULL;
    RoboHeartDRV8836* _directionMotor = NULL;

    void calculateDiff(int timeout_ms = 500);
    bool isCalibrated(int timeout_ms = 500);

};

#endif