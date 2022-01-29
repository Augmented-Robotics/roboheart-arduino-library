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
};

#endif