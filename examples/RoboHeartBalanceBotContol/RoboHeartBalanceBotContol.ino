/* This example shows how to balance and control
 * the Balancing Bot made with the RoboHeart
 * board.
 *
 * Setup for the Augmented Reality Smartphone App
 * is provided. It establishes connection
 * through Bluetooth.
 */

#include <RoboHeart.h>
#include <RoboHeartBLE.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart();

#define Kp 20
#define Kd 0.001
#define Ki 40

#define CONTROL_PERIOD_US 100.0

#define PID_CONTROL_PRESCALER 5
#define DC_CONTROL_PRESCALER 15
#define WD_TIMER_PRESCALER 3000
#define STATISTICS_PRESCALER DC_CONTROL_PRESCALER * 4

#define MOTOR_FORWARD false
#define MOTOR_REVERSE true
#define MOTOR_RIGHT true
#define MOTOR_LEFT false

float prevAngleDeg = 0;
float motorPower = 0;
float errorSum = 0;
float currentAngleDeg = 0;
float offsetMotorPower = 0;
float targetAngleDeg = 0.;
float offsetAngleDeg = 0.;
float turnMotorA = 1.;
float turnMotorB = 1.;

unsigned long pidControlTick = 0;
unsigned long dcControlTick = 0;
unsigned long statisticsTick = 0;
unsigned long wdTimerTick = 0;

unsigned long prevTimeIntervalMS = 0;

/* BLE params */
static uint8_t packageBle[4] = {0x11, 0x22, 0x33, 0x44};
volatile bool bleDeviceConnected = false;
bool bleConnUpdated = false;

void setupDirection(bool dir) {
    if (dir == MOTOR_FORWARD) {
        targetAngleDeg = offsetAngleDeg - 5;
        offsetMotorPower = 50;
    } else if (dir == MOTOR_REVERSE) {
        targetAngleDeg = offsetAngleDeg + 5;
        offsetMotorPower = -50;
    }
}

void setupTurn(int dir) {
    if (dir == MOTOR_LEFT) {
        turnMotorA = 1.5;
        turnMotorB = 1.;
    } else if (dir == MOTOR_RIGHT) {
        turnMotorA = 1.;
        turnMotorB = 1.5;
    }
}

void resetControl() {
    offsetMotorPower = 0;
    targetAngleDeg = offsetAngleDeg;
    turnMotorA = 1.;
    turnMotorB = 1.;
}

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

void bleDisconnected() {
    bleConnUpdated = false;
    bleDeviceConnected = false;
}

void bleConnected() {
    bleConnUpdated = false;
    bleDeviceConnected = true;
}

float processAngle(float angle) {
    // handle -180 after crossing 180
    if (angle < -90) {
        return angle = 360 + angle;
    }
    return angle;
}

void tick() {
    // reduce code to minimal in order to avoid errors
    pidControlTick++;
    dcControlTick++;
    statisticsTick++;
    wdTimerTick++;
}

void processPinInterrupt() {
    offsetAngleDeg = heart.mpu.getAngleX();
    targetAngleDeg = offsetAngleDeg;
}

InterfaceBLE ble = InterfaceBLE();
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_PERIOD_US, Serial);

void setup() {
    Serial.begin(115200);

    // initialize with or without request for IMU automatic calibration
    heart.begin(false);

    // use manual offsets (taken from previous calibrations)
    heart.mpu.setGyroOffsets(-1.76, -0.07, -0.9);
    heart.mpu.setAccOffsets(0.04, -0.00, 0.11);

    //  // print calculated offsets
    //  Serial.print("Offsets: gx ");
    //  Serial.print(heart.mpu.getGyroXoffset());
    //  Serial.print(" gy ");
    //  Serial.print(heart.mpu.getGyroYoffset());
    //  Serial.print(" gz ");
    //  Serial.print(heart.mpu.getGyroZoffset());
    //  Serial.print(" ax ");
    //  Serial.print(heart.mpu.getAccXoffset());
    //  Serial.print(" ay ");
    //  Serial.print(heart.mpu.getAccYoffset());
    //  Serial.print(" az ");
    //  Serial.println(heart.mpu.getAccZoffset());

    // ble configuration
    ble.configure(packageBle, sizeof(packageBle));

    ble.setServerCallbacks(bleConnected, bleDisconnected);
    ble.setCharacteristicsCallbacks(onWriteMotorControl, NULL, NULL);
    ble.startServiceAdvertising();

    pinMode(BUTTON_ROBOHEART, INPUT);
    attachInterrupt(BUTTON_ROBOHEART, processPinInterrupt, FALLING);

    // Resolve false triggering of button during flashing
    // TODO: remove in RH rev 0.3
    delay(100);
    offsetAngleDeg = 0;
    targetAngleDeg = 0;

    prevTimeIntervalMS = millis();
    timer.enable();

    Serial.println("Init finished");
}

void loop() {
    heart.beat();

    // Perform PID control every CONTROL_PERIOD_US*PID_CONTROL_PRESCALER
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

    // Perform Motor control every CONTROL_PERIOD_US*DC_CONTROL_PRESCALER
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

    // Activate safety timer every CONTROL_PERIOD_US*WD_TIMER_PRESCALER
    if (wdTimerTick >= WD_TIMER_PRESCALER) {
        wdTimerTick = 0;
        resetControl();
    }

    if (!bleDeviceConnected && !bleConnUpdated) {
        // BLE disconnecting
        ble.startServiceAdvertising();
        Serial.println("Disconnected, start advertising");
        bleConnUpdated = true;
        motorPower = 0;
    }

    if (bleDeviceConnected && !bleConnUpdated) {
        // BLE connecting
        Serial.println("Connected");
        bleConnUpdated = true;
    }

    // Print statistics every CONTROL_PERIOD_US*STATISTICS_PRESCALER
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
