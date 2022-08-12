/*!
 *  @file RoboHeartTimer.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#ifndef RoboHeartTimer_h
#define RoboHeartTimer_h

#include <Arduino.h>

class PeriodicTimer {
   public:
    PeriodicTimer(void (*callback)(void), uint64_t timerPeriodMicroSec);
    PeriodicTimer(void (*callback)(void), uint64_t timerPeriodMicroSec,
                  Stream& debug);
    void start();
    void stop();
    void setTimePeriod(uint64_t timerPeriodMicroSec);

   private:
    Stream* _debug = NULL;
    hw_timer_t* timer;
};

#endif  // RoboHeartTimer_h