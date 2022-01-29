/*!
 *  @file RoboHeartDRV8836.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEARTDRV8836_H
#define _ROBOHEARTDRV8836_H

#include "pins.h"
#include <Arduino.h>

class RoboHeartDRV8836
{
    public:
        RoboHeartDRV8836();
        ~RoboHeartDRV8836();
        void begin(int modePin, int in1Pin, int in2Pin, int nsleepPin);
        void coast();
        void forward(uint8_t speed);
        void reverse(uint8_t speed);
        void brake();
        uint8_t getSpeed();

    private:
        int _modePin = -1;
        int _in1Pin = -1;
        int _in2Pin = -1;
        int _nsleepPin = -1;
        uint8_t _speed = 0;
};

#endif
