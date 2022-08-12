/*!
 *  @file RoboHeartStepperMotor.h
 *
 * 	Arduino library for the RoboHeart.
 *
 */
#ifndef RoboHeartStepperMotor_h
#define RoboHeartStepperMotor_h

#include <Arduino.h>

#include "RoboHeartDRV8836.h"
#include "pins_RoboHeart.h"

#define STEPPER_MOTOR_MAX_STEPS 8

typedef enum { STEPPER_FORWARD = 0, STEPPER_REVERSE } StepperDirectionType;

class RoboHeartStepperMotor {
   public:
    RoboHeartStepperMotor();
    RoboHeartStepperMotor(Stream& debug);
    ~RoboHeartStepperMotor();
    void executeHalfStep(int cmd, StepperDirectionType direction);
    void begin(RoboHeartDRV8836& motorIN1, RoboHeartDRV8836& motorIN2);
    void stepForward();
    void stepReverse();

   private:
    Stream* _debug = NULL;
    RoboHeartDRV8836* _motorIN1 =
        NULL;  // TODO: configure to use any motor, maybe motorC?
    RoboHeartDRV8836* _motorIN2 = NULL;
    int command = 0;
};

#endif
