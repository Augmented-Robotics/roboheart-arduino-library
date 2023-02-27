/* This example shows how to control the car with
 * the Smartphone Augmented Reality App through Bluetooth.
 *
 * RoboHeart installation instructions can be found here:
 * https://www.instructables.com/How-to-Install-Your-RoboHeart-Hercules-Into-a-RC-C/
 * 
 * Created  30/01/2022
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <RoboHeartBLE.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart(Serial);

const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2
const float MIN_BATTERY_VOLTAGE = 3.7; // Min voltage at which the motors stop working
const float BATTERY_RANGE = 0.5;


// Example of the package that can be transmited with BLE
static uint8_t blePackage[4] = {0x11, 0x22, 0x33, 0x44};

// BLE status flags
bool bleDeviceConnected = false;
bool bleNewStatusReceived = false;

#define WD_TIMER_PERIOD_US \
    300000  // Monitor that the new information is
            // received within that time frame

// Flag to indicate if the timer is triggered
int wdTimerTriggered = 0;

// Action when no response received within WD_TIMER_PERIOD_US
void wdTimerCallback() { wdTimerTriggered++; }

// Timer gets activated when no response received within WD_TIMER_PERIOD_US
PeriodicTimer watchdogTimer =
    PeriodicTimer(wdTimerCallback, WD_TIMER_PERIOD_US, Serial);

InterfaceBLE ble = InterfaceBLE(Serial);

// When user sends data to characterstic repsonsible for the Motor Control
void onWriteMotorControl(std::string value) {
    if (value.length() == 3) {
        watchdogTimer.stop();
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
        watchdogTimer.start();
    } else {
        Serial.print("Received invalid control message with length: ");
        Serial.println(value.length());
        Serial.println("Doing nothing.");
    }
}

// Callback for device disconnected event
void bleDisconnected() {
    bleNewStatusReceived = true;
    bleDeviceConnected = false;
    Serial.println("device disconnected.");
    heart.motorA.sleep(true);
    heart.motorB.sleep(true);
    heart.motorC.sleep(true);
}

// Callback for device connected event
void bleConnected() {
    bleNewStatusReceived = true;
    bleDeviceConnected = true;
    Serial.println("device connected.");
    heart.motorA.sleep(false);
    heart.motorB.sleep(false);
    heart.motorC.sleep(false);
}

int battery_percent(){
   float raw_value = analogRead(BATTERY_PIN);
   float voltage_level = ((raw_value  * 3.3)/ MAX_ANALOG_VAL )* 2 ; // multiply the analogRead value by 2x to get the true battery
   //Serial.println(voltage_level);
   float battery_fraction = voltage_level / MAX_BATTERY_VOLTAGE;
   float voltage_range = voltage_level - MIN_BATTERY_VOLTAGE;
   int percent = (voltage_range / BATTERY_RANGE) * 100;
   return percent;
}


void setup() {
    Serial.begin(115200);

    // Set up the RoboHeart
    heart.begin();

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

    Serial.println("RoboHeart BLE Demo");
}

void loop() {
    // Give computing time to the RoboHeart
    heart.beat();
    int percent = battery_percent();
    // Send some information to the ble
    if (bleDeviceConnected) {
        blePackage[0]++;
        ble.sendNotifyChar2(blePackage);
        ble.sendNotifyChar3((uint8_t *)(&percent));
        delay(
            3);  // bluetooth stack will go into congestion, if too many packets
                 // are sent, in 6 hours test we were able to go as low as 3ms
    }

    // Disconnecting
    if (!bleDeviceConnected && bleNewStatusReceived) {
        bleNewStatusReceived = false;
        delay(500);  // give the bluetooth stack the chance to get things ready
        ble.startServiceAdvertising();
        Serial.println("start advertising");
        heart.motorA.sleep(true);
        heart.motorB.sleep(true);
        heart.motorC.sleep(true);
    }

    // Connecting
    if (bleDeviceConnected && bleNewStatusReceived) {
        bleNewStatusReceived = false;
        // do stuff here on connecting
    }

    // Watchdog timer
    if (wdTimerTriggered > 0) {
        watchdogTimer.stop();

        Serial.print("Safety timer activated: ");
        Serial.println(wdTimerTriggered);

        wdTimerTriggered = 0;

        Serial.println("Stopping motors");
        MotorMSGType motorMessage = {0, 0, 0};

        heart.handleMotorMessage(motorMessage);
    }
}