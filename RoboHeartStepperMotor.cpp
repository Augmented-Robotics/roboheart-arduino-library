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

RoboHeartStepperMotor::RoboHeartStepperMotor() { _debug = NULL; }

RoboHeartStepperMotor::RoboHeartStepperMotor(Stream& debug) : _debug(&debug) {}

RoboHeartStepperMotor::~RoboHeartStepperMotor() {}

void RoboHeartStepperMotor::begin(RoboHeartDRV8836* motor0,
                                  RoboHeartDRV8836* motor1) {
    _motor0 = motor0;
    _motor1 = motor1;
}

void RoboHeartStepperMotor::executeHalfStep(int cmd,
                                            StepperDirection_t direction) {
    if (_motor0 == NULL || _motor1 == NULL) {
        DEBUG_LN_IDENTIFIER("Configure the struct with begin() first");
        return;
    }

    int motor0MaxDuty = _motor0->getMaxDutyCycle();
    int motor1MaxDuty = _motor1->getMaxDutyCycle();

    float entry_ratio = 1.;
    float exit_ratio = 1.;
    float peak_ratio = 1.;

    // Creating smoother transitions with PWM
    // thanks to the inductance and resistance of the coils
    // if (direction == STEPPER_FORWARD){
    //     entry_ratio = 57./64.;
    //     exit_ratio = 41./64.;
    // } else if (direction == STEPPER_REVERSE){
    //     entry_ratio = 41./64.;
    //     exit_ratio = 57./64.;
    // }

    // 8 half-steps
    // In more details:     Quick Start for Beginners to Drive a Stepper Motor
    // Link: https://www.nxp.com/docs/en/application-note/AN2974.pdf

    switch (cmd) {
        case 0:
            /* A+ */
            _motor0->forward(peak_ratio * motor0MaxDuty);
            /* B0 */
            _motor1->sleep(false);
            break;
        case 1:
            /* A+ */
            _motor0->forward(exit_ratio * motor0MaxDuty);
            /* B+ */
            _motor1->forward(entry_ratio * motor1MaxDuty);
            break;
        case 2:
            /* A0 */
            _motor0->sleep(false);
            /* B+ */
            _motor1->forward(peak_ratio * motor1MaxDuty);
            break;
        case 3:
            /* A- */
            _motor0->reverse(entry_ratio * motor0MaxDuty);
            /* B+ */
            _motor1->forward(exit_ratio * motor1MaxDuty);
            break;
        case 4:
            /* A- */
            _motor0->reverse(peak_ratio * motor0MaxDuty);
            /* B0 */
            _motor1->sleep(false);
            break;
        case 5:
            /* A- */
            _motor0->reverse(exit_ratio * motor0MaxDuty);
            /* B- */
            _motor1->reverse(entry_ratio * motor1MaxDuty);
            break;
        case 6:
            /* A0 */
            _motor0->sleep(false);
            /* B- */
            _motor1->reverse(peak_ratio * motor1MaxDuty);
            break;
        case 7:
            /* A+ */
            _motor0->forward(entry_ratio * motor0MaxDuty);
            /* B- */
            _motor1->reverse(exit_ratio * motor1MaxDuty);
            break;

        default:
            DEBUG_IDENTIFIER("Error: Received invalid command: ");
            DEBUG_LN(cmd);
            break;
    }
}

void RoboHeartStepperMotor::step_forward() {
    command++;
    if (command >= STEPPER_MOTOR_MAX_STEPS) {
        command = 0;
    }

    executeHalfStep(command, STEPPER_FORWARD);
}

void RoboHeartStepperMotor::step_reverse() {
    command--;
    if (command < 0) {
        command = STEPPER_MOTOR_MAX_STEPS - 1;
    }

    executeHalfStep(command, STEPPER_REVERSE);
}
