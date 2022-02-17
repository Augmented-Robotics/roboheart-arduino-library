/*!
 *  @file RoboHeart.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEART_H
#define _ROBOHEART_H

#include "pins.h"
#include <Arduino.h>

#include "RoboHeartDRV8836.h"
#include <Wire.h>
#include <MPU6050_light.h>



  /**
     * @brief 
     * AR Link Commands
     * 
     * Higly inefficient structure, kept for backwards compatibility
     * 
     * command possible values:
     0 - stop              - 0000 0000
    1 - forward           - 0000 0001
    2 - backwards         - 0000 0010
    3 - left              - 0000 0011
    4 - right             - 0000 0100
    5 - forward left      - 0000 0101
    6 - forward right     - 0000 0110
    7 - backwards left    - 0000 0111
    8 - backwards right   - 0000 1000
    *
    * speed                - rare wheels power
    * steering_power       - front wheels turning power
    */

    typedef struct 
    { 
        uint8_t command; 
        uint8_t speed; 
        uint8_t steering_power; 
    } Motor_MSG_t;



class RoboHeart
{
    public:
        RoboHeart();
        ~RoboHeart();

        bool begin();
        void beat();

        void motor0_coast();   
        void motor0_sleep(bool sleep=true);
        void motor0_reverse(uint8_t speed);
        void motor0_forward(uint8_t speed);
        void motor0_brake();

        void motor1_coast();
        void motor1_sleep(bool sleep=true);
        void motor1_reverse(uint8_t speed);
        void motor1_forward(uint8_t speed);
        void motor1_brake();

        void motor2_coast();
        void motor2_sleep(bool sleep=true);
        void motor2_reverse(uint8_t speed);
        void motor2_forward(uint8_t speed);
        void motor2_brake();

        void handleMotorMessage(Motor_MSG_t, char* response);

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

    private:
        RoboHeartDRV8836 motor0;
        RoboHeartDRV8836 motor1;
        RoboHeartDRV8836 motor2;
        
        
        
};

extern MPU6050 mpu;

#endif