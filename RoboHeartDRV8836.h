/*!
 *  @file RoboHeartDRB8836.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */
#ifndef RoboHeartDRV8836_h
#define RoboHeartDRV8836_h

#include <Arduino.h>

#include "pins_RoboHeart.h"

class RoboHeartDRV8836 {
   public:
    RoboHeartDRV8836();
    RoboHeartDRV8836(Stream& debug);
    ~RoboHeartDRV8836();
    void begin(int in1Pin, int in2Pin, int nsleepPin, int in1Channel = 0,
               int in2Channel = 1);
    void sleep(bool sleep = true);
    void coast();
    void forward(int speed);
    void reverse(int speed);
    void configPWM(int freq = 100000, int resolution = 8);
    void brake();
    int getSpeed();
    int getMaxDutyCycle();

   private:
    Stream* _debug = NULL;
    int _in1Pin = -1;
    int _in2Pin = -1;
    int _nSleepPin = -1;
    int _speed = 0;
    int _pwmFreq = -1;
    int _pwmResolution = -1;
    int _pwmMaxDutyCycle = 256;

    int _in1Channel = 0;  // TODO: Remove on the new arduino-esp32 release
    int _in2Channel = 1;
};

#endif
