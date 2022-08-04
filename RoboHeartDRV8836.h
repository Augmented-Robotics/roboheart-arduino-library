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
        RoboHeartDRV8836(Stream& debug);
        ~RoboHeartDRV8836();
        void begin(int modePin, int in1Pin, int in2Pin, int nsleepPin);
        void sleep(bool sleep=true);
        void coast();
        void forward(int speed);
        void reverse(int speed);
        void configPWM(int freq = 100000, int resolution = 8);
        void brake();
        int getSpeed();
        int getMaxDutyCycle();

    private:
        Stream* _debug;     
        int _modePin = -1;
        int _in1Pin = -1;
        int _in2Pin = -1;
        int _nsleepPin = -1;
        int _speed = 0;
        int _pwmFreq = -1;
        int _pwmResolution = -1;
        int _pwmMaxDutyCycle = 256;
        
};

#endif
