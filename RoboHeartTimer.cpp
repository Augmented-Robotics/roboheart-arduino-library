/*!
 *  @file RoboHeartTimer.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeartTimer.h"

#define FILE_IDENTIFIER \
    "TIMER"  // Define identifier before including DebuggerMsgs.h
#include "DebuggerMsgs.h"

PeriodicTimer::PeriodicTimer(void (*callback)(void),
                             uint64_t timerPeriodMicroSec)
    : timer(timerBegin(3)) {
    timerAttachInterrupt(timer, callback);
    setTimePeriod(timerPeriodMicroSec);
    stop();
}

PeriodicTimer::PeriodicTimer(void (*callback)(void),
                             uint64_t timerPeriodMicroSec, Stream& debug)
    : timer(timerBegin(3)), _debug(&debug) {
    timerAttachInterrupt(timer, callback);
    setTimePeriod(timerPeriodMicroSec);
    stop();
}

void PeriodicTimer::setTimePeriod(uint64_t timerPeriodMicroSec) {
    
    timerStop(timer);
    timerWrite(timer, timerPeriodMicroSec);
    timerRestart(timer);
    timerStart(timer);
}

void PeriodicTimer::start() {
    timerStart(timer);
}

void PeriodicTimer::stop() {
    timerStop(timer);
    timerRestart(timer);
}