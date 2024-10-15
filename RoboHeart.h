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
#define TRESHOLD 0.3  //treshold in degrees/s
#define PERIOD_US 200

#define MOTOR_A_CHANNEL1 0
#define MOTOR_A_CHANNEL2 1
#define MOTOR_B_CHANNEL1 2
#define MOTOR_B_CHANNEL2 3
#define MOTOR_C_CHANNEL1 4
#define MOTOR_C_CHANNEL2 5

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2

#define CALIBRATION_COUNTDOWN 5
#define ACCELEROMETER_CUTOFF 32
#define NEW_IMU_DATA_BIT 1

#define LSM6DS3_I2C_ADDR 0x6A // or 0x6B - Address is defined in Library

typedef struct {
    int command;
    int speed;
    int steeringPower;
} MotorMSGType;

typedef struct {
  int16_t rX;
  int16_t rY;
  int16_t rZ;
  int16_t aX;
  int16_t aY;
  int16_t aZ;
} tIMUdata;

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
    void resetGyro();

    RoboHeartStepperMotor stepper;
    RoboHeartDRV8836 motorA;
    RoboHeartDRV8836 motorB;
    RoboHeartDRV8836 motorC;
    static LSM6DS3 imu;
    
    static EventGroupHandle_t xIMUeventGroup;
    static SemaphoreHandle_t xVarMutex;
    static float _rotationX;
    static float _rotationY;
    static float _rotationZ;
    static bool tick;
    static tIMUdata imuData;

    float getTemperatureC();
    float getTemperatureF();
    void setPWM(int motor, int freq, int pwm);
    
    static void computeEulerRates(float omega_x, float omega_y, float omega_z, float phi, float theta, float* dphi, float* dtheta, float* dpsi);
    static void rungeKutta4(float* phi, float* theta, float* psi, float omega_x, float omega_y, float omega_z, float dt);
    static void rungeKutta2(float* phi, float* theta, float* psi, float omega_x, float omega_y, float omega_z, float dt);

    static void IMUgetDataTask(void *pvParameter);
    static void IMUcalculateDataTask(void *pvParameter);

   private:
    Stream* _debug = NULL;
    RoboHeartDRV8836* _turnMotor = NULL;
    RoboHeartDRV8836* _directionMotor = NULL;

};

#endif