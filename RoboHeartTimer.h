/*!
 *  @file RoboHeart.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEARTTIMER_H
#define _ROBOHEARTTIMER_H

#include <Arduino.h>

class WatchdogTimer
{
    public:
        WatchdogTimer(void (*callback)(void), uint64_t timerPeriodMicroSec);
        WatchdogTimer(void (*callback)(void), uint64_t timerPeriodMicroSec, Stream& debug);
        void enable();
        void disable();
        void setTimePeriod(uint64_t timerPeriodMicroSec);

    private:
        Stream* _debug;  
        hw_timer_t * timer;   
        
};


#endif // _ROBOHEARTTIMER_H