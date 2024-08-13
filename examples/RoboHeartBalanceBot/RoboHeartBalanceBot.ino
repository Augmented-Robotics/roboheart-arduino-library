/* This example shows how to balance
 * the Balancing Bot made with the RoboHeart
 * board.
 * 
 * Use button to set the stable vertical position
 * of the Balancing Bot.
 * 
 *  
 * Building a One-board balancing bot instructions can be found here:
 * https://www.instructables.com/A-One-board-Balancing-Bot-With-a-RoboHeart-Electro/
 * 
 * Created  30/01/2022
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart(Serial);

// PID controller parameters
#define Kp 20
#define Kd 0
#define Ki 40

#define CONTROL_TICK_PERIOD_US 100.0

#define PID_CONTROL_PRESCALER \
    5  // Control ticks passing before PID motorPower Calculation
#define DC_CONTROL_PRESCALER 15  // Control ticks passing before DC Motor Control
#define STATISTICS_PRESCALER \
    DC_CONTROL_PRESCALER *   \
        4  // Control ticks passing before all the debug printing is performed

// Motor control parameters
float prevAngleDeg = 0;      // Store previos angle
float motorPower = 0;        // Calculated Power to be supplied to the motors
float errorSum = 0;          // For Integral part of the PID controller
float currentAngleDeg = 0;   // Current Angle
float targetAngleDeg = 0.;   // Target Angle for the Balancing bot
float offsetAngleDeg = 0.;   // Stable vertical position angle

// Track the timer activation for each task individually
unsigned long pidControlTick = 0;
unsigned long dcControlTick = 0;
unsigned long statisticsTick = 0;

// Store previous time
unsigned long prevTimeIntervalMS = 0;

// handle -180 after crossing 180 Degrees
float processAngle(float angle) {
    // handle -180 after crossing 180
    if (angle > 180) {
        return angle - 360;
    }
    return angle;
}

// Periodic timer executes the control ticks
void tick() {
    // reduce code to minimal in order to avoid errors
    pidControlTick++;
    dcControlTick++;
    statisticsTick++;
}

// When Button is pressed the current angle is saved
// and later used to indicate stable vertical position
// of the Balancing Bot.
void processPinInterrupt() {
    heart.resetGyro();
    errorSum = 0;
}

// Periodic timer executes the control ticks
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_TICK_PERIOD_US, Serial);

void setup() {
    Serial.begin(115200);

    // Initialize RoboHeart with or without request for IMU automatic
    // calibration
    heart.begin();

    //Run IMU calibration and calculation of absolute rotation
    heart.setAutomaticRotation();

    // Configure Button which is used to set the
    // stable vertical position of the Balancing Bot.
    pinMode(BUTTON_ROBOHEART, INPUT);
    attachInterrupt(BUTTON_ROBOHEART, processPinInterrupt, FALLING);

    delay(100); // Resolve false triggering of button during flashing
    offsetAngleDeg = 0;
    targetAngleDeg = 0;

    // Save current time and start timer
    prevTimeIntervalMS = millis();
    timer.start();

    Serial.println("RoboHeart Balance Bot Control Demo");
}

void loop() {

    // Perform PID control every CONTROL_TICK_PERIOD_US*PID_CONTROL_PRESCALER
    if (pidControlTick >= PID_CONTROL_PRESCALER) {
        unsigned long curTimeIntervalMS = millis();
        pidControlTick = 0;
        currentAngleDeg = processAngle(heart.getRotationX());

        float error = currentAngleDeg - targetAngleDeg;
        errorSum = constrain(errorSum + error, -Kp * 50, Kp * 50);

        float sampleTimeS = .001 * (curTimeIntervalMS - prevTimeIntervalMS);

        // Calculate and sum output for P, I and D values
        motorPower = Kp * (error) + Ki * (errorSum)*sampleTimeS -
                     Kd * (currentAngleDeg - prevAngleDeg) / sampleTimeS;
        prevAngleDeg = currentAngleDeg;
        prevTimeIntervalMS = curTimeIntervalMS;
    }

    // Perform Motor control every CONTROL_TICK_PERIOD_US*DC_CONTROL_PRESCALER
    if (dcControlTick >= DC_CONTROL_PRESCALER) {
        dcControlTick = 0;
        if (motorPower > 0) {
            heart.motorA.reverse(motorPower);
            heart.motorB.reverse(motorPower);
        } else if (motorPower < 0) {
            heart.motorA.forward(-motorPower);
            heart.motorB.forward(-motorPower);
        }
    }

    // Print statistics every CONTROL_TICK_PERIOD_US*STATISTICS_PRESCALER
    if (statisticsTick >= STATISTICS_PRESCALER) {
        statisticsTick = 0;
        Serial.print(" P :");
        Serial.print(motorPower);
        Serial.print(" A :");
        Serial.print(currentAngleDeg);
        Serial.print(" AT :");
        Serial.print(targetAngleDeg);
        Serial.print(" ES :");
        Serial.println(errorSum);
    }

    delay(3);
}
