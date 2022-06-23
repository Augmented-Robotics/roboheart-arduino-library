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

typedef enum {
    M_DRIVER_A = 0,
    M_DRIVER_B,
    M_DRIVER_C,
    M_DRIVERS
} Motor_DRV_t;

typedef enum {
    // MOTOR 0
    PWM_MA1_CH = 0,
    PWM_MA2_CH,

    //MOTOR 1
    PWM_MB1_CH,
    PWM_MB2_CH,   

    //MOTOR 2
    PWM_MC1_CH,
    PWM_MC2_CH,
    PWM_MX_CHANNELS
} PWM_Channel_t;

class RoboHeartDRV8836
{
    public:
        RoboHeartDRV8836();
        RoboHeartDRV8836(Stream& debug);
        ~RoboHeartDRV8836();
        void begin(int modePin, int in1Pin, int in2Pin, int nsleepPin, Motor_DRV_t mDRV);
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
        int _pwmMaxDutyCycle = 0;
        PWM_Channel_t _in1Channel = PWM_MX_CHANNELS;
        PWM_Channel_t _in2Channel = PWM_MX_CHANNELS;
        
};

#endif
