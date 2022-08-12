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
    : timer(timerBegin(0, 80, true)) {
    timerAttachInterrupt(timer, callback, true);
    setTimePeriod(timerPeriodMicroSec);
    stop();
}

PeriodicTimer::PeriodicTimer(void (*callback)(void),
                             uint64_t timerPeriodMicroSec, Stream& debug)
    : timer(timerBegin(0, 80, true)), _debug(&debug) {
    timerAttachInterrupt(timer, callback, true);
    setTimePeriod(timerPeriodMicroSec);
    stop();
}

void PeriodicTimer::setTimePeriod(uint64_t timerPeriodMicroSec) {
    bool startAfterConfig = false;

    if (timerStarted(timer)) {
        startAfterConfig = true;
        timerStop(timer);
    }

    timerAlarmWrite(timer, timerPeriodMicroSec, true);

    timerRestart(timer);

    if (startAfterConfig) {
        timerStart(timer);
    }
}

void PeriodicTimer::start() {
    timerRestart(timer);
    timerAlarmEnable(timer);
    timerStart(timer);
}

void PeriodicTimer::stop() {
    timerStop(timer);
    timerRestart(timer);
}