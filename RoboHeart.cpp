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

#include "Arduino.h"
#include "RoboHeart.h"

RoboHeart::RoboHeart(void)
{

}

RoboHeart::~RoboHeart(void)
{

}

bool RoboHeart::begin()
{
    // MOTOR SETUP
    
    pinMode(GPIO_M0A_PH_IN1, OUTPUT);//  PHASE/IN1
    digitalWrite(GPIO_M0A_PH_IN1,LOW);
    pinMode(GPIO_M0B_EN_IN2, OUTPUT);//  ENABLE/IN2
    digitalWrite(GPIO_M0B_EN_IN2,LOW); 
    pinMode(GPIO_M0_MODE, OUTPUT);//  MODE
    // MODE PIN LOW --> IN/IN MODE
    // MODE PIN HIGH --> PHASE/ENABLE MODE
    digitalWrite(GPIO_M0_MODE,LOW);
    pinMode(GPIO_M0_SLEEP, OUTPUT);//  nSLEEP
    digitalWrite(GPIO_M0_SLEEP,HIGH);

    pinMode(GPIO_M1A_PH_IN1, OUTPUT);//  PHASE/IN1
    digitalWrite(GPIO_M1A_PH_IN1,LOW);
    pinMode(GPIO_M1B_EN_IN2, OUTPUT);//  ENABLE/IN2
    digitalWrite(GPIO_M1B_EN_IN2,LOW); 
    pinMode(GPIO_M1_MODE, OUTPUT);//  MODE
    // MODE PIN LOW --> IN/IN MODE
    // MODE PIN HIGH --> PHASE/ENABLE MODE
    digitalWrite(GPIO_M1_MODE,LOW);
    pinMode(GPIO_M1_SLEEP, OUTPUT);//  nSLEEP
    digitalWrite(GPIO_M1_SLEEP,HIGH);

     pinMode(GPIO_M2A_PH_IN1, OUTPUT);//  PHASE/IN1
    digitalWrite(GPIO_M2A_PH_IN1,LOW);
    pinMode(GPIO_M2B_EN_IN2, OUTPUT);//  ENABLE/IN2
    digitalWrite(GPIO_M2B_EN_IN2,LOW); 
    pinMode(GPIO_M2_MODE, OUTPUT);//  MODE
    // MODE PIN LOW --> IN/IN MODE
    // MODE PIN HIGH --> PHASE/ENABLE MODE
    digitalWrite(GPIO_M2_MODE,LOW);
    pinMode(GPIO_M2_SLEEP, OUTPUT);//  nSLEEP
    digitalWrite(GPIO_M2_SLEEP,HIGH);

    return true;
}

void RoboHeart::beat()
{
}



void RoboHeart::motor0_coast()
{
    digitalWrite(GPIO_M0A_PH_IN1,LOW);
    digitalWrite(GPIO_M0B_EN_IN2,LOW); 
}

void RoboHeart::motor0_reverse()
{
    digitalWrite(GPIO_M0A_PH_IN1,LOW);
    digitalWrite(GPIO_M0B_EN_IN2,HIGH); 
}

void RoboHeart::motor0_forward()
{
    digitalWrite(GPIO_M0A_PH_IN1,HIGH);
    digitalWrite(GPIO_M0B_EN_IN2,LOW); 
}

void RoboHeart::motor0_brake()
{
    digitalWrite(GPIO_M0A_PH_IN1,HIGH);
    digitalWrite(GPIO_M0B_EN_IN2,HIGH); 
}


void RoboHeart::motor1_coast()
{
    digitalWrite(GPIO_M1A_PH_IN1,LOW);
    digitalWrite(GPIO_M1B_EN_IN2,LOW); 
}

void RoboHeart::motor1_reverse()
{
    digitalWrite(GPIO_M1A_PH_IN1,LOW);
    digitalWrite(GPIO_M1B_EN_IN2,HIGH); 
}

void RoboHeart::motor1_forward()
{
    digitalWrite(GPIO_M1A_PH_IN1,HIGH);
    digitalWrite(GPIO_M1B_EN_IN2,LOW); 
}

void RoboHeart::motor1_brake()
{
    digitalWrite(GPIO_M1A_PH_IN1,HIGH);
    digitalWrite(GPIO_M1B_EN_IN2,HIGH); 
}



void RoboHeart::motor2_coast()
{
    digitalWrite(GPIO_M2A_PH_IN1,LOW);
    digitalWrite(GPIO_M2B_EN_IN2,LOW); 
}

void RoboHeart::motor2_reverse()
{
    digitalWrite(GPIO_M2A_PH_IN1,LOW);
    digitalWrite(GPIO_M2B_EN_IN2,HIGH); 
}

void RoboHeart::motor2_forward()
{
    digitalWrite(GPIO_M2A_PH_IN1,HIGH);
    digitalWrite(GPIO_M2B_EN_IN2,LOW); 
}

void RoboHeart::motor2_brake()
{
    digitalWrite(GPIO_M2A_PH_IN1,HIGH);
    digitalWrite(GPIO_M2B_EN_IN2,HIGH); 
}