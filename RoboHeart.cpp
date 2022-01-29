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

RoboHeart::RoboHeart(void)
{
   
    //motor0 = new RoboHeartDRV8836();

    
    //motor1 = new RoboHeartDRV8836();

   
    //motor2 = new RoboHeartDRV8836();
    
}

RoboHeart::~RoboHeart(void)
{
}

bool RoboHeart::begin()
{
    
   
    // MOTOR SETUP
    
    motor0.begin(GPIO_M0_MODE, GPIO_M0A_PH_IN1, GPIO_M0B_EN_IN2, GPIO_M0_SLEEP);
    motor1.begin(GPIO_M1_MODE, GPIO_M1A_PH_IN1, GPIO_M1B_EN_IN2, GPIO_M1_SLEEP);
    motor2.begin(GPIO_M2_MODE, GPIO_M2A_PH_IN1, GPIO_M2B_EN_IN2, GPIO_M2_SLEEP);

    return true;
}

void RoboHeart::beat()
{
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