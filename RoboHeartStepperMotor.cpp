/*!
 *  @file RoboHeartStepperMotor.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeartStepperMotor.h"

#define FILE_IDENTIFIER \
    "STEPPER"  // Define identifier before including DebuggerMsgs.h
#include "DebuggerMsgs.h"

RoboHeartStepperMotor::RoboHeartStepperMotor() {}

RoboHeartStepperMotor::RoboHeartStepperMotor(Stream& debug) : _debug(&debug) {}

RoboHeartStepperMotor::~RoboHeartStepperMotor() {}

void RoboHeartStepperMotor::begin(RoboHeartDRV8836& motorIN1,
                                  RoboHeartDRV8836& motorIN2) {
    _motorIN1 = &motorIN1;
    _motorIN2 = &motorIN2;
}

void RoboHeartStepperMotor::executeHalfStep(int cmd,
                                            StepperDirectionType direction) {
    RETURN_WARN_IF_EQUAL(_motorIN1, NULL)

    int motorAMaxDuty = _motorIN1->getMaxDutyCycle();
    int motorBMaxDuty = _motorIN2->getMaxDutyCycle();

    float entryRatio = 1.;
    float exitRatio = 1.;
    float peakRatio = 1.;

    // Creating smoother transitions with PWM
    // thanks to the inductance and resistance of the coils
    // if (direction == STEPPER_FORWARD){
    //     entryRatio = 57./64.;
    //     exitRatio = 41./64.;
    // } else if (direction == STEPPER_REVERSE){
    //     entryRatio = 41./64.;
    //     exitRatio = 57./64.;
    // }

    // 8 half-steps
    // In more details:     Quick Start for Beginners to Drive a Stepper Motor
    // Link: https://www.nxp.com/docs/en/application-note/AN2974.pdf

    switch (cmd) {
        case 0:
            /* A+ */
            _motorIN1->forward(peakRatio * motorAMaxDuty);
            /* B0 */
            _motorIN2->sleep(false);
            break;
        case 1:
            /* A+ */
            _motorIN1->forward(exitRatio * motorAMaxDuty);
            /* B+ */
            _motorIN2->forward(entryRatio * motorBMaxDuty);
            break;
        case 2:
            /* A0 */
            _motorIN1->sleep(false);
            /* B+ */
            _motorIN2->forward(peakRatio * motorBMaxDuty);
            break;
        case 3:
            /* A- */
            _motorIN1->reverse(entryRatio * motorAMaxDuty);
            /* B+ */
            _motorIN2->forward(exitRatio * motorBMaxDuty);
            break;
        case 4:
            /* A- */
            _motorIN1->reverse(peakRatio * motorAMaxDuty);
            /* B0 */
            _motorIN2->sleep(false);
            break;
        case 5:
            /* A- */
            _motorIN1->reverse(exitRatio * motorAMaxDuty);
            /* B- */
            _motorIN2->reverse(entryRatio * motorBMaxDuty);
            break;
        case 6:
            /* A0 */
            _motorIN1->sleep(false);
            /* B- */
            _motorIN2->reverse(peakRatio * motorBMaxDuty);
            break;
        case 7:
            /* A+ */
            _motorIN1->forward(entryRatio * motorAMaxDuty);
            /* B- */
            _motorIN2->reverse(exitRatio * motorBMaxDuty);
            break;

        default:
            DEBUG_IDENTIFIER("Error: Received invalid command: ");
            DEBUG_LN(cmd);
            break;
    }
}

void RoboHeartStepperMotor::stepForward() {
    command++;
    if (command >= STEPPER_MOTOR_MAX_STEPS) {
        command = 0;
    }

    executeHalfStep(command, STEPPER_FORWARD);
}

void RoboHeartStepperMotor::stepReverse() {
    command--;
    if (command < 0) {
        command = STEPPER_MOTOR_MAX_STEPS - 1;
    }

    executeHalfStep(command, STEPPER_REVERSE);
}
