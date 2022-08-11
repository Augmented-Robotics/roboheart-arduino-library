/*!
 *  @file RoboHeartTimer.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeartTimer.h"

#define DEBUG_TIMER(x)                       \
    {                                        \
        if (_debug != NULL) {                \
            _debug->print("[TIMER_DEBUG] "); \
            _debug->print(x);                \
        }                                    \
    }
#define DEBUG_LN_TIMER(x)                    \
    {                                        \
        if (_debug != NULL) {                \
            _debug->print("[TIMER_DEBUG] "); \
            _debug->println(x);              \
        }                                    \
    }
#define DEBUG(x)              \
    {                         \
        if (_debug != NULL) { \
            _debug->print(x); \
        }                     \
    }
#define DEBUG_LN(x)             \
    {                           \
        if (_debug != NULL) {   \
            _debug->println(x); \
        }                       \
    }

PeriodicTimer::PeriodicTimer(void (*callback)(void),
                             uint64_t timerPeriodMicroSec)
    : timer(timerBegin(0, 80, true)), _debug(NULL) {
    timerAttachInterrupt(timer, callback, true);
    setTimePeriod(timerPeriodMicroSec);
    disable();
}

PeriodicTimer::PeriodicTimer(void (*callback)(void),
                             uint64_t timerPeriodMicroSec, Stream& debug)
    : timer(timerBegin(0, 80, true)), _debug(&debug) {
    timerAttachInterrupt(timer, callback, true);
    setTimePeriod(timerPeriodMicroSec);
    disable();
}

void PeriodicTimer::setTimePeriod(uint64_t timerPeriodMicroSec) {
    bool start_after_config = false;

    if (timerStarted(timer)) {
        start_after_config = true;
        timerStop(timer);
    }

    timerAlarmWrite(timer, timerPeriodMicroSec, true);

    timerRestart(timer);

    if (start_after_config) {
        timerStart(timer);
    }
}

void PeriodicTimer::enable() {
    timerRestart(timer);
    timerAlarmEnable(timer);
    timerStart(timer);
}

void PeriodicTimer::disable() {
    timerStop(timer);
    timerRestart(timer);
}