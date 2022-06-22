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
    Wire.begin();
     // MPU6050 SETUP
    Serial.print("MPU6050 status: ");
    mpu.setAddress(MPU6050_I2C_ADDR);
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
    
    motor0.begin(GPIO_MA_MODE, GPIO_MA_PH_IN1, GPIO_MA_EN_IN2, GPIO_MA_SLEEP, M_DRIVER_A);
    motor1.begin(GPIO_MB_MODE, GPIO_MB_PH_IN1, GPIO_MB_EN_IN2, GPIO_MB_SLEEP, M_DRIVER_B);
    motor2.begin(GPIO_MC_MODE, GPIO_MC_PH_IN1, GPIO_MC_EN_IN2, GPIO_MC_SLEEP, M_DRIVER_C);

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

void RoboHeart::motor0_sleep(bool sleep)
{
    motor0.sleep(sleep); 
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


void RoboHeart::motor1_sleep(bool sleep)
{
    motor1.sleep(sleep); 
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


void RoboHeart::motor2_sleep(bool sleep)
{
    motor2.sleep(sleep); 
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


 void RoboHeart::handleMotorMessage(Motor_MSG_t motormsg, char* response)
 {
     switch(motormsg.command)
     {
         case 1:
            //forward
            motor2_forward(motormsg.speed);
            sprintf (response, "forward");
            break;
         case 2:
            //reverse
            motor2_reverse(motormsg.speed);
            sprintf (response, "reverse");
            break;
         case 3:
            //right
            sprintf (response, "right");
            motor1_forward(motormsg.steering_power);
            break;
         case 4:
            //left
            sprintf (response, "left");
            motor1_reverse(motormsg.steering_power);
            break;
         case 5:
            //forward and right
            sprintf (response, "forward and right");
            motor2_forward(motormsg.speed);
            motor1_forward(motormsg.steering_power);
            break;
         case 6:
            //forward and left
            sprintf (response, "forward and left");
            motor2_forward(motormsg.speed);
            motor1_reverse(motormsg.steering_power);
            break;
         case 7:
            //reverse and right
            sprintf (response, "reverse and right");
            motor2_reverse(motormsg.speed);
            motor1_forward(motormsg.steering_power);
            break;
        case 8:
            //reverse and left
            sprintf (response, "reverse and left");
            motor2_reverse(motormsg.speed);
            motor1_reverse(motormsg.steering_power);
            break;
        case 0:
            //STOP
            sprintf (response, "STOP");
            motor2.brake();
            motor1.brake();
            
            break;  
        default:
            //STOP
            sprintf (response, "ERROR: %d", motormsg.command);
            motor2.brake();
            motor1.brake();
            break;    
     }

        
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
