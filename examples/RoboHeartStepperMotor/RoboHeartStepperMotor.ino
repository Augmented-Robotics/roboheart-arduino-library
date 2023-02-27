/* This example shows how to control the built-in
 * Stepper motor drivers.
 * 
 * Created  30/01/2022
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart(Serial);

#define CONTROL_TICK_PERIOD_US 100  // Control tick period in micro-seconds
#define STEPPER_CONTROL_PRESCALER \
    25  // Control ticks passing before Stepper Motor Control
#define MOTOR_STEPS_FULL_ROTATION \
    230  // Stepper steps before achieving one full rotation

// Control is achieved through half-step mode
#define MOTOR_CTRL_STEPS_FULL_ROTATION (2 * MOTOR_STEPS_FULL_ROTATION)

int steps = 0;
int controlTick = 0;

#define STEPPER_FORWARD true
#define STEPPER_REVERSE false

bool direction = STEPPER_FORWARD;

void tick() { controlTick++; }

// Periodic timer executes the control ticks
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_TICK_PERIOD_US, Serial);

void setup() {
    Serial.begin(115200);

    // Set up the RoboHeart
    heart.begin();

    // start the timer
    timer.start();

    Serial.println("RoboHeart StepperMotor Demo");
}

unsigned long prevTimeMS = 0;

void loop() {
    // Stepper Motor Control
    if (controlTick >= STEPPER_CONTROL_PRESCALER) {
        controlTick = 0;
        steps++;
        if (direction == STEPPER_FORWARD) {
            heart.stepper.stepForward();
        } else if (direction == STEPPER_REVERSE) {
            heart.stepper.stepReverse();
        } else {
            Serial.println("Direction has invalid value");
        }
    }

    // Full rotation achieved
    if (steps >= MOTOR_CTRL_STEPS_FULL_ROTATION) {
        timer.stop();

        unsigned long currentTimeMS = millis();
        Serial.print("Full rotation done |");
        Serial.print(" Time: ");
        Serial.print(currentTimeMS - prevTimeMS);
        Serial.print(" Steps: ");
        Serial.println(steps);

        // change direction
        if (direction == STEPPER_FORWARD) {
            direction = STEPPER_REVERSE;
        } else {
            direction = STEPPER_FORWARD;
        }

        prevTimeMS = currentTimeMS;
        steps = 0;
        timer.start();
    }
}