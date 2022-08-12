/* This example shows how to control the built-in
 * Stepper motor drivers
 */

#include <RoboHeart.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart();

#define CONTROL_PERIOD_US 100
#define STEPPER_CONTROL_PRESCALER 25
#define MOTOR_STEPS_FULL_ROTATION 230

// Control is achieved through half-step mode
#define MOTOR_CTRL_STEPS_FULL_ROTATION (2 * MOTOR_STEPS_FULL_ROTATION)

int steps = 0;
int controlTick = 0;

#define STEPPER_FORWARD true
#define STEPPER_REVERSE false

bool direction = STEPPER_FORWARD;

void tick() { controlTick++; }

PeriodicTimer timer = PeriodicTimer(tick, CONTROL_PERIOD_US, Serial);

void setup() {
    Serial.begin(115200);

    heart.begin();

    timer.enable();
}

unsigned long prevTimeMS = 0;

void loop() {
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

    if (steps >= MOTOR_CTRL_STEPS_FULL_ROTATION) {
        timer.disable();

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
        timer.enable();
    }
}