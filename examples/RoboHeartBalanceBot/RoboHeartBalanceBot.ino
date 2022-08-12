/* This example shows how to balance
 * the Balancing Bot made with the RoboHeart
 * board.
 * 
 * Use button to set the stable vertical position
 * of the Balancing Bot.
 */

#include <RoboHeart.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart();

// PID controller parameters
#define Kp 20
#define Kd 0.001
#define Ki 40

#define CONTROL_TICK_PERIOD_US 100.0

#define PID_CONTROL_PRESCALER \
    5  // Control ticks pasing before PID motorPower Calculation
#define DC_CONTROL_PRESCALER 15  // Control ticks pasing before DC Motor Control
#define STATISTICS_PRESCALER \
    DC_CONTROL_PRESCALER *   \
        4  // Control ticks pasing before all the debug printing is performed

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
    if (angle < -90) {
        return angle = 360 + angle;
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
    offsetAngleDeg = heart.mpu.getAngleX();
    targetAngleDeg = offsetAngleDeg;
}

// Periodic timer executes the control ticks
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_TICK_PERIOD_US, Serial);

void setup() {
    Serial.begin(115200);

    // Initialize RoboHeart with or without request for IMU automatic
    // calibration
    bool mpuRequestCalibration = true;
    heart.begin(mpuRequestCalibration);

    // One can set mpuRequestCalibration = false and
    // use manual offsets (taken from previous calibrations)
    // heart.mpu.setGyroOffsets(-1.76, -0.07, -0.9);
    // heart.mpu.setAccOffsets(0.04, -0.00, 0.11);

    // use manual offsets (taken from previous calibrations)
    heart.mpu.setGyroOffsets(-1.76, -0.07, -0.9);
    heart.mpu.setAccOffsets(0.04, -0.00, 0.11);

    // print calculated offsets
    char offsets_msg[200];
    sprintf(offsets_msg, "Offsets Gyro gx: %f, gy: %f gz: %f",
            heart.mpu.getGyroXoffset(), heart.mpu.getGyroYoffset(),
            heart.mpu.getGyroZoffset());
    Serial.println(offsets_msg);
    sprintf(offsets_msg, "Offsets Accel ax: %f, ay: %f az: %f",
            heart.mpu.getAccXoffset(), heart.mpu.getAccYoffset(),
            heart.mpu.getAccZoffset());
    Serial.println(offsets_msg);

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
    // Give computing time to the RoboHeart
    heart.beat();

    // Perform PID control every CONTROL_TICK_PERIOD_US*PID_CONTROL_PRESCALER
    if (pidControlTick >= PID_CONTROL_PRESCALER) {
        unsigned long curTimeIntervalMS = millis();
        pidControlTick = 0;
        currentAngleDeg = processAngle(heart.mpu.getAngleX());

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
