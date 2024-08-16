/* This example shows how to balance and control
 * the Balancing Bot made with the RoboHeart
 * board.
 *
 * Setup for the Augmented Reality Smartphone App
 * is provided. It establishes connection
 * through Bluetooth.
 * 
 * Use button to set the stable vertical position
 * of the Balancing Bot.
 * 
 * Building a One-board balancing bot instructions can be found here:
 * https://www.instructables.com/A-One-board-Balancing-Bot-With-a-RoboHeart-Electro/
 * 
 * Created  16/08/2024
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <RoboHeartBLE.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart(Serial);

// PID controller parameters.
// You should change parameters based on your setup.
// You can use our another example RoboHeartPIDFittingExample to try set parameters to stabilise.
#define Kp 20
#define Kd 0
#define Ki 40

#define CONTROL_TICK_PERIOD_US 100.0  // Control tick period in micro-seconds

#define PID_CONTROL_PRESCALER \
    5  // Control ticks passing before PID motorPower Calculation
#define DC_CONTROL_PRESCALER 15  // Control ticks passing before DC Motor Control
#define WD_TIMER_PRESCALER \
    3000  // Monitor that the new information is received within that time frame
#define STATISTICS_PRESCALER \
    DC_CONTROL_PRESCALER *   \
        4  // Control ticks passing before all the debug printing is performed

#define MOTOR_FORWARD false
#define MOTOR_REVERSE true
#define MOTOR_RIGHT true
#define MOTOR_LEFT false

// Motor control parameters
float prevAngleDeg = 0;      // Store previos angle
float motorPower = 0;        // Calculated Power to be supplied to the motors
float errorSum = 0;          // For Integral part of the PID controller
float currentAngleDeg = 0;   // Current Angle
float offsetMotorPower = 0;  // Motor Power Bias (for FORWARD/BACKWARD motions)
float targetAngleDeg = 0.;   // Target Angle for the Balancing bot
float offsetAngleDeg = 0.;   // Stable vertical position angle
float turnMotorA = 1.;       // Coefficient for Motor A to generate Turn motion
float turnMotorB = 1.;       // Coefficient for Motor B to generate Turn motion

// Track the timer activation for each task individually
unsigned long pidControlTick = 0;
unsigned long dcControlTick = 0;
unsigned long statisticsTick = 0;
unsigned long wdTimerTick = 0;

// Store previous time
unsigned long prevTimeIntervalMS = 0;

// Example of the package that can be transmited with BLE
static uint8_t packageBle[4] = {0x11, 0x22, 0x33, 0x44};

// BLE status flags
bool bleDeviceConnected = false;
bool bleNewStatusReceived = false;

// Configure configuration for Forward/Backward motion
void setupDirection(bool dir) {
    if (dir == MOTOR_FORWARD) {
        targetAngleDeg = offsetAngleDeg - 5;
        offsetMotorPower = 50;
    } else if (dir == MOTOR_REVERSE) {
        targetAngleDeg = offsetAngleDeg + 5;
        offsetMotorPower = -50;
    }
}

// Configure configuration for Turning motion
void setupTurn(int dir) {
    if (dir == MOTOR_LEFT) {
        turnMotorA = 1.5;
        turnMotorB = 1.;
    } else if (dir == MOTOR_RIGHT) {
        turnMotorA = 1.;
        turnMotorB = 1.5;
    }
}

// Reset control values
void resetControl() {
    offsetMotorPower = 0;
    targetAngleDeg = offsetAngleDeg;
    turnMotorA = 1.;
    turnMotorB = 1.;
}

// When user sends data to characterstic repsonsible for the Motor Control
void onWriteMotorControl(std::string value) {
    if (value.length() == 3) {
        wdTimerTick = 0;

        MotorMSGType motorMessage = {value[0], 3 * int(value[1]),
                                     3 * int(value[2])};

        switch (motorMessage.command) {
            case 1:
                // forward
                setupDirection(MOTOR_FORWARD);
                turnMotorA = 1.;
                turnMotorB = 1.;
                break;
            case 2:
                // reverse
                setupDirection(MOTOR_REVERSE);
                turnMotorA = 1.;
                turnMotorB = 1.;
                break;
            case 3:
                // right
                break;
            case 4:
                // left
                break;
            case 5:
                // forward and right
                setupDirection(MOTOR_FORWARD);
                setupTurn(MOTOR_RIGHT);
                break;
            case 6:
                // forward and left
                setupDirection(MOTOR_FORWARD);
                setupTurn(MOTOR_LEFT);
                break;
            case 7:
                // reverse and right
                setupDirection(MOTOR_REVERSE);
                setupTurn(MOTOR_RIGHT);
                break;
            case 8:
                // reverse and left
                setupDirection(MOTOR_REVERSE);
                setupTurn(MOTOR_LEFT);
                break;
            case 0:
                // stop
                resetControl();
                break;
        }
    }
}

// Callback for device disconnected event
void bleDisconnected() {
    bleNewStatusReceived = true;
    bleDeviceConnected = false;
    Serial.println("device disconnected.");
}

// Callback for device connected event
void bleConnected() {
    bleNewStatusReceived = true;
    bleDeviceConnected = true;
    Serial.println("device connected.");
}

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
    wdTimerTick++;
}

// When Button is pressed the current angle is saved
// and later used to indicate stable vertical position
// of the Balancing Bot.
void processPinInterrupt() {
    heart.resetGyro();
    errorSum = 0;
}

InterfaceBLE ble = InterfaceBLE();

// Periodic timer executes the control ticks
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_TICK_PERIOD_US, Serial);

void setup() {
    Serial.begin(115200);

    // Initialize RoboHeart with or without request for IMU automatic
    // calibration
    heart.begin();
    
    Serial.println("Please let RoboHeart in stable position...");
    
    //Run IMU calibration and calculation of absolute rotation
    heart.setAutomaticRotation();

    // BLE configuration

    // Setting Callbacks are not mandatory but provides useful functionality
    // Connection and disconnection callbacks
    ble.setServerCallbacks(bleConnected, bleDisconnected);
    // Callbacks for the write events for each characteristic
    ble.setCharacteristicsCallbacks(onWriteMotorControl, NULL, NULL);

    // Call begin to finish all the Bluetooth configurations
    ble.begin();

    // Start advartising so that the App can connect
    ble.startServiceAdvertising();

    // Configure Button which is used to set the
    // stable vertical position of the Balancing Bot.
    pinMode(BUTTON_ROBOHEART, INPUT);
    attachInterrupt(BUTTON_ROBOHEART, processPinInterrupt, FALLING);

    delay(100);  // Resolve false triggering of button during flashing
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
            heart.motorA.reverse(turnMotorA * (motorPower + offsetMotorPower));
            heart.motorB.reverse(turnMotorB * (motorPower + offsetMotorPower));
        } else if (motorPower < 0) {
            heart.motorA.forward(turnMotorA * (-motorPower - offsetMotorPower));
            heart.motorB.forward(turnMotorB * (-motorPower - offsetMotorPower));
        }
    }

    // Activate safety timer every CONTROL_TICK_PERIOD_US*WD_TIMER_PRESCALER
    if (wdTimerTick >= WD_TIMER_PRESCALER) {
        wdTimerTick = 0;
        resetControl();
    }

    // Disconnecting
    if (!bleDeviceConnected && bleNewStatusReceived) {
        bleNewStatusReceived = false;
        ble.startServiceAdvertising();
        Serial.println("Start advertising");
        motorPower = 0;
    }

    // Connecting
    if (bleDeviceConnected && bleNewStatusReceived) {
        bleNewStatusReceived = false;
        // do stuff here on connecting
    }

    // Print statistics every CONTROL_TICK_PERIOD_US*STATISTICS_PRESCALER
    if (statisticsTick >= STATISTICS_PRESCALER) {
        statisticsTick = 0;
        Serial.print(" P :");
        Serial.print(motorPower);
        Serial.print(" OP :");
        Serial.print(offsetMotorPower);
        Serial.print(" A :");
        Serial.print(currentAngleDeg);
        Serial.print(" AT :");
        Serial.print(targetAngleDeg);
        Serial.print(" ES :");
        Serial.println(errorSum);
    }

    delay(3);
}
