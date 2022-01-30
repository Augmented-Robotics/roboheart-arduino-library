/*!
 *  @file GEENYmodem.cpp
 *
 *  @mainpage Arduino library for the RoboHeart Hardware.
 *
 *  @section intro_sec Introduction
 *
 * 	Arduino library for the RoboHeart.
 *
 *  @section dependencies Dependencies
 *  This library depends on the tinyGSM library, the HTTPClient library, and the PubSub library.
 
 *
 *  @section author Author
 *
 *  Augmented Robotics
 *
 * 	@section license License
 *
 * 	MIT (see license.txt)
 */


#include "RoboHeart.h"


RoboHeart::RoboHeart()
{
    
}

RoboHeart::~RoboHeart(void)
{
    //delete mpu;
}

bool RoboHeart::begin()
{
     // MPU6050 SETUP
    Serial.print("MPU6050 status: ");
    byte status = mpu.begin();
    
    Serial.println(status);
    if(status == 0)
    {
        Serial.println("Calculating offsets, do not move MPU6050");
        delay(1000);
        mpu.calcOffsets(true,true); // gyro and accelero
        Serial.println("Done!\n");
    }
    
   
    // MOTOR SETUP
    
    motor0.begin(GPIO_M0_MODE, GPIO_M0A_PH_IN1, GPIO_M0B_EN_IN2, GPIO_M0_SLEEP);
    motor1.begin(GPIO_M1_MODE, GPIO_M1A_PH_IN1, GPIO_M1B_EN_IN2, GPIO_M1_SLEEP);
    motor2.begin(GPIO_M2_MODE, GPIO_M2A_PH_IN1, GPIO_M2B_EN_IN2, GPIO_M2_SLEEP);

    return true;
}

void RoboHeart::beat()
{
     mpu.update();
}



void RoboHeart::motor0_coast()
{
    motor0.coast(); 
}

void RoboHeart::motor0_reverse(uint8_t speed)
{
    motor0.reverse(speed);
}

void RoboHeart::motor0_forward(uint8_t speed)
{
    motor0.forward(speed);
}

void RoboHeart::motor0_brake()
{
    motor0.brake();
}



void RoboHeart::motor1_coast()
{
    motor1.coast(); 
}

void RoboHeart::motor1_reverse(uint8_t speed)
{
    motor1.reverse(speed);
}

void RoboHeart::motor1_forward(uint8_t speed)
{
    motor1.forward(speed);
}

void RoboHeart::motor1_brake()
{
    motor1.brake();
}


void RoboHeart::motor2_coast()
{
    motor2.coast(); 
}

void RoboHeart::motor2_reverse(uint8_t speed)
{
    motor2.reverse(speed);
}

void RoboHeart::motor2_forward(uint8_t speed)
{
    motor2.forward(speed);
}

void RoboHeart::motor2_brake()
{
    motor2.brake();
}

float RoboHeart::getTemp(){ return mpu.getTemp(); }

float RoboHeart::getAccX(){ return mpu.getAccX(); }
float RoboHeart::getAccY(){ return mpu.getAccY(); }
float RoboHeart::getAccZ(){ return mpu.getAccZ(); }

float RoboHeart::getGyroX(){ return mpu.getGyroX(); }
float RoboHeart::getGyroY(){ return mpu.getGyroY(); }
float RoboHeart::getGyroZ(){ return mpu.getGyroZ(); }

float RoboHeart::getAccAngleX(){ return mpu.getAccAngleX(); }
float RoboHeart::getAccAngleY(){ return mpu.getAccAngleY(); }

float RoboHeart::getAngleX(){ return mpu.getAngleX(); }
float RoboHeart::getAngleY(){ return mpu.getAngleY(); }
float RoboHeart::getAngleZ(){ return mpu.getAngleZ(); }

MPU6050 mpu(Wire);
