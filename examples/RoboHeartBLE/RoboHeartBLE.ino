#include <RoboHeart.h>
#include <RoboHeartTimer.h>
#include <RoboHeartBLE.h>


RoboHeart heart = RoboHeart();

static uint8_t package_ble[4] = {0x11, 0x22, 0x33, 0x44};

volatile bool bleDeviceConnected = false;
bool bleNewStatusReceived = false;

// Safety timer
#define WD_TIMER_PERIOD_US   300000
volatile int wd_timer_triggered = 0;
 
void wdTimerCallback() {
  wd_timer_triggered++;
}

WatchdogTimer watchdogTimer = WatchdogTimer(wdTimerCallback, WD_TIMER_PERIOD_US, Serial);

// BLE
InterfaceBLE ble = InterfaceBLE(Serial);

void onWriteMotorControl(std::string value) {
  if (value.length() == 3) {
    watchdogTimer.disable();
    Serial.println("*********");
    Serial.print("New value: ");
    for (int i = 0; i < value.length(); i++)
    {
      Serial.print(int(value[i]));
      Serial.print(" ");
    }
    Serial.println();
  
    Motor_MSG_t motor_message = { value[0], 3*int(value[1]), 3*int(value[2])};
    Serial.println("MOTOR MESSAGE:");
    Serial.print("command: ");
    Serial.println(motor_message.command);
    Serial.print("speed: ");
    Serial.println(motor_message.speed);
    Serial.print("steering_power: ");
    Serial.println(motor_message.steering_power);
  
    char response[20];
    heart.handleMotorMessage(motor_message, &response[0]);
    Serial.print("response: ");
    Serial.println(response);
  
    Serial.println("*********");
    watchdogTimer.enable();
  }
}

void BLEdisconnected() {
  bleNewStatusReceived = true;
  bleDeviceConnected = false;
  Serial.println("device disconnected.");
  heart.motor0_sleep(true);
  heart.motor1_sleep(true);
  heart.motor2_sleep(true);
}

void BLEconnected() {
  bleNewStatusReceived = true;
  bleDeviceConnected = true;
  Serial.println("device connected.");
  heart.motor0_sleep(false);
  heart.motor1_sleep(false);
  heart.motor2_sleep(false);
}

void setup() {
  Serial.begin(115200);

  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();

  // set up the RoboHeart
  heart.begin();

  // ble configuration
  ble.configure(package_ble, sizeof(package_ble));
 
  ble.setServerCallbacks(BLEconnected, BLEdisconnected);
  ble.setCharacteristicsCallbacks(onWriteMotorControl, NULL, NULL);
  ble.StartServiceAdvertising();

  Serial.println("RH setup finished");
  
}

void loop() {
  //give computing time to the RoboHeart
  heart.beat();

  // send some information to the ble
  if (bleDeviceConnected) {
    package_ble[0]++;
    ble.sendNotifyChar2(package_ble);
    delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  
  // disconnecting
  if (!bleDeviceConnected && bleNewStatusReceived) {
    bleNewStatusReceived = false;
    delay(500); // give the bluetooth stack the chance to get things ready
    ble.StartServiceAdvertising();
    Serial.println("start advertising");
    heart.motor0_sleep(true);
    heart.motor1_sleep(true);
    heart.motor2_sleep(true);
  }

  // connecting
  if (bleDeviceConnected && bleNewStatusReceived) {
    bleNewStatusReceived = false;
    // do stuff here on connecting
  }

  // watchdog timer
  if (wd_timer_triggered > 0){
    watchdogTimer.disable();

    Serial.print("Safety timer activated: ");
    Serial.println(wd_timer_triggered);
    
    wd_timer_triggered = 0;

    Serial.println("Stopping motors");
    Motor_MSG_t motor_message = {0, 0, 0};

    char response[20];
    heart.handleMotorMessage(motor_message, &response[0]);
    
  }
}