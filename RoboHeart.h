/*!
 *  @file RoboHeart.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEART_H
#define _ROBOHEART_H

#include "pins.h"

class RoboHeart
{
    public:
        RoboHeart();
        ~RoboHeart();

        bool begin();
        void beat();

        void motor0_coast();
        void motor0_reverse();
        void motor0_forward();
        void motor0_brake();

        void motor1_coast();
        void motor1_reverse();
        void motor1_forward();
        void motor1_brake();

        void motor2_coast();
        void motor2_reverse();
        void motor2_forward();
        void motor2_brake();
};

#endif