/*!
 *  @file RoboHeart.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEART_H
#define _ROBOHEART_H

#include <Arduino.h>
#include "pins.h"
#include "RoboHeartDRV8836.h"

class RoboHeart
{
    public:
        RoboHeart();
        ~RoboHeart();

        bool begin();
        void beat();

        void motor0_coast();
        void motor0_reverse(uint8_t speed);
        void motor0_forward(uint8_t speed);
        void motor0_brake();

        void motor1_coast();
        void motor1_reverse(uint8_t speed);
        void motor1_forward(uint8_t speed);
        void motor1_brake();

        void motor2_coast();
        void motor2_reverse(uint8_t speed);
        void motor2_forward(uint8_t speed);
        void motor2_brake();

    private:
        RoboHeartDRV8836 motor0;
        RoboHeartDRV8836 motor1;
        RoboHeartDRV8836 motor2;
        
};

#endif