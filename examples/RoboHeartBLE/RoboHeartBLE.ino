/* This example shows how to control the car with
 * the Smartphone Augmented Reality App through Bluetooth.
 */

#include <RoboHeart.h>
#include <RoboHeartBLE.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart();

static uint8_t blePackage[4] = {0x11, 0x22, 0x33, 0x44};

volatile bool bleDeviceConnected = false;
bool bleNewStatusReceived = false;

// Safety timer
#define WD_TIMER_PERIOD_US 300000
volatile int wdTimerTriggered = 0;

void wdTimerCallback() { wdTimerTriggered++; }

PeriodicTimer watchdogTimer =
    PeriodicTimer(wdTimerCallback, WD_TIMER_PERIOD_US, Serial);

// BLE
InterfaceBLE ble = InterfaceBLE(Serial);

void onWriteMotorControl(std::string value) {
    if (value.length() == 3) {
        watchdogTimer.disable();
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++) {
            Serial.print(int(value[i]));
            Serial.print(" ");
        }
        Serial.println();

        MotorMSGType motorMessage = {value[0], 3 * int(value[1]),
                                     3 * int(value[2])};
        Serial.println("MOTOR MESSAGE:");
        Serial.print("command: ");
        Serial.println(motorMessage.command);
        Serial.print("speed: ");
        Serial.println(motorMessage.speed);
        Serial.print("steeringPower: ");
        Serial.println(motorMessage.steeringPower);

        char *response = heart.handleMotorMessage(motorMessage);
        Serial.print("response: ");
        Serial.println(response);

        Serial.println("*********");
        watchdogTimer.enable();
    }
}

void bleDisconnected() {
    bleNewStatusReceived = true;
    bleDeviceConnected = false;
    Serial.println("device disconnected.");
    heart.motorA.sleep(true);
    heart.motorB.sleep(true);
    heart.motorC.sleep(true);
}

void bleConnected() {
    bleNewStatusReceived = true;
    bleDeviceConnected = true;
    Serial.println("device connected.");
    heart.motorA.sleep(false);
    heart.motorB.sleep(false);
    heart.motorC.sleep(false);
}

void setup() {
    Serial.begin(115200);

    // set up the RoboHeart
    heart.begin();

    // ble configuration
    ble.configure(blePackage, sizeof(blePackage));

    ble.setServerCallbacks(bleConnected, bleDisconnected);
    ble.setCharacteristicsCallbacks(onWriteMotorControl, NULL, NULL);
    ble.startServiceAdvertising();

    Serial.println("RH setup finished");
}

void loop() {
    // give computing time to the RoboHeart
    heart.beat();

    // send some information to the ble
    if (bleDeviceConnected) {
        blePackage[0]++;
        ble.sendNotifyChar2(blePackage);
        delay(
            3);  // bluetooth stack will go into congestion, if too many packets
                 // are sent, in 6 hours test i was able to go as low as 3ms
    }

    // disconnecting
    if (!bleDeviceConnected && bleNewStatusReceived) {
        bleNewStatusReceived = false;
        delay(500);  // give the bluetooth stack the chance to get things ready
        ble.startServiceAdvertising();
        Serial.println("start advertising");
        heart.motorA.sleep(true);
        heart.motorB.sleep(true);
        heart.motorC.sleep(true);
    }

    // connecting
    if (bleDeviceConnected && bleNewStatusReceived) {
        bleNewStatusReceived = false;
        // do stuff here on connecting
    }

    // watchdog timer
    if (wdTimerTriggered > 0) {
        watchdogTimer.disable();

        Serial.print("Safety timer activated: ");
        Serial.println(wdTimerTriggered);

        wdTimerTriggered = 0;

        Serial.println("Stopping motors");
        MotorMSGType motorMessage = {0, 0, 0};

        heart.handleMotorMessage(motorMessage);
    }
}