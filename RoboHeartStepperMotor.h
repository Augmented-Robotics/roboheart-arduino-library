/*!
 *  @file RoboHeartStepperMotor.h
 *
 *
 *	MIT license (see license.txt)
 */
#ifndef _ROBOHEARTSTEPPERMOTOR_H
#define _ROBOHEARTSTEPPERMOTOR_H

#include "pins.h"
#include <Arduino.h>
#include "RoboHeartDRV8836.h"

#define STEPPER_MOTOR_MAX_STEPS 8

typedef enum {
    STEPPER_FORWARD = 0,
    STEPPER_REVERSE
} StepperDirection_t;

class RoboHeartStepperMotor
{
    public:
        RoboHeartStepperMotor();
        RoboHeartStepperMotor(Stream& debug);
        ~RoboHeartStepperMotor();
        void executeHalfStep(int cmd, StepperDirection_t direction);
        void begin(RoboHeartDRV8836* motor0, RoboHeartDRV8836* motor1);
        void step_forward();
        void step_reverse();
    private:
        Stream* _debug; 
        RoboHeartDRV8836* _motor0 = NULL; //TODO: configure to use any motor, maybe motor2?
        RoboHeartDRV8836* _motor1 = NULL;
        int command = 0;
        
};

#endif
