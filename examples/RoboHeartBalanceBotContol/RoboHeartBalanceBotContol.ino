#include <RoboHeart.h>
#include <RoboHeartTimer.h>
#include <RoboHeartBLE.h>

RoboHeart heart = RoboHeart();

#define Kp  20
#define Kd  0.001
#define Ki  40

#define BUTTON_PIN 0

#define CONTROL_PERIOD_US   100.0

#define PID_CONTROL_PRESCALER   5   
#define DC_CONTROL_PRESCALER 15
#define WD_TIMER_PRESCALER   3000
#define STATISTICS_PRESCALER DC_CONTROL_PRESCALER*4

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
float turnmotor0 = 1.;
float turnmotor1 = 1.;

unsigned long pidControlTick = 0;
unsigned long dcControlTick = 0;
unsigned long statisticsTick = 0;
unsigned long wdTimerTick = 0;

unsigned long prevTimeIntervalMS = 0;

/* BLE params */
static uint8_t packageBle[4] = {0x11, 0x22, 0x33, 0x44};
volatile bool bleDeviceConnected = false;
bool bleConnUpdated = false;

void setup_direction(bool dir){
  if (dir == MOTOR_FORWARD){
    targetAngleDeg = offsetAngleDeg - 5;
    offsetMotorPower = 50;
  } else if (dir == MOTOR_REVERSE) {
    targetAngleDeg = offsetAngleDeg + 5;
    offsetMotorPower = -50;    
  }
}

void setup_turn(int dir){
  if (dir == MOTOR_LEFT){
    turnmotor0 = 1.5;
    turnmotor1 = 1.;
  } else if (dir == MOTOR_RIGHT) {
    turnmotor0 = 1.;
    turnmotor1 = 1.5;
  }
}

void reset_control() {
    offsetMotorPower = 0;
    targetAngleDeg = offsetAngleDeg;
    turnmotor0 = 1.;
    turnmotor1 = 1.;
}

void onWriteMotorControl(std::string value) {
  if (value.length() == 3) {
    wdTimerTick = 0;
  
    Motor_MSG_t motor_message = { value[0], 3*int(value[1]), 3*int(value[2])};

    switch(motor_message.command)
    {
       case 1:
        //forward
        setup_direction(MOTOR_FORWARD);
        turnmotor0 = 1.;
        turnmotor1 = 1.;
        break;
       case 2:
        //reverse
        setup_direction(MOTOR_REVERSE);
        turnmotor0 = 1.;
        turnmotor1 = 1.;
        break;
       case 3:
        //right
        break;
       case 4:
        //left
        break;
       case 5:
        //forward and right
        setup_direction(MOTOR_FORWARD);
        setup_turn(MOTOR_RIGHT);
        break;
       case 6:
        //forward and left
        setup_direction(MOTOR_FORWARD);
        setup_turn(MOTOR_LEFT);
        break;
       case 7:
        //reverse and right
        setup_direction(MOTOR_REVERSE);
        setup_turn(MOTOR_RIGHT);
        break;
       case 8:
        //reverse and left
        setup_direction(MOTOR_REVERSE);
        setup_turn(MOTOR_LEFT);
        break;
       case 0:
        //stop
        reset_control();
        break;
    }
  }
}

void BLEdisconnected() {
  bleConnUpdated = false;
  bleDeviceConnected = false;
}

void BLEconnected() {
  bleConnUpdated = false;
  bleDeviceConnected = true;
}

float process_angle(float angle){
  // handle -180 after crossing 180
  if (angle < -90){
    return angle = 360+angle;
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

void processPinInterrupt()
{
  offsetAngleDeg = heart.getAngleX();
  targetAngleDeg = offsetAngleDeg;
}
 
InterfaceBLE ble = InterfaceBLE();
PeriodicTimer timer = PeriodicTimer(tick, CONTROL_PERIOD_US, Serial);

void setup() {
  Serial.begin(115200);

  // initialize with or without request for IMU automatic calibration
  heart.begin(false);

  // use manual offsets (taken from previous calibrations)
  heart.setGyroOffsets(-1.76, -0.07, -0.9);
  heart.setAccOffsets(0.04, -0.00, 0.11);

//  // print calculated offsets  
//  Serial.print("Offsets: gx ");
//  Serial.print(heart.getGyroXoffset());
//  Serial.print(" gy ");
//  Serial.print(heart.getGyroYoffset());
//  Serial.print(" gz ");
//  Serial.print(heart.getGyroZoffset());
//  Serial.print(" ax ");
//  Serial.print(heart.getAccXoffset());
//  Serial.print(" ay ");
//  Serial.print(heart.getAccYoffset());
//  Serial.print(" az ");
//  Serial.println(heart.getAccZoffset());

  // ble configuration
  ble.configure(packageBle, sizeof(packageBle));
 
  ble.setServerCallbacks(BLEconnected, BLEdisconnected);
  ble.setCharacteristicsCallbacks(onWriteMotorControl, NULL, NULL);
  ble.StartServiceAdvertising();

  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(BUTTON_PIN, processPinInterrupt, FALLING);

  // Resolve false triggering of button during flashing
  // TODO: remove in RH rev 0.3
  delay (100);
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
    currentAngleDeg = process_angle(heart.getAngleX());
    
    float error = currentAngleDeg - targetAngleDeg;
    errorSum = constrain(errorSum + error , -Kp*50, Kp*50);  

    float sampleTimeS = .001*(curTimeIntervalMS - prevTimeIntervalMS);
    
    // Calculate and sum output for P, I and D values
    motorPower = Kp*(error) + Ki*(errorSum)*sampleTimeS- Kd*(currentAngleDeg-prevAngleDeg)/sampleTimeS;
    prevAngleDeg = currentAngleDeg;
    prevTimeIntervalMS = curTimeIntervalMS;  
  }

  // Perform Motor control every CONTROL_PERIOD_US*DC_CONTROL_PRESCALER 
  if (dcControlTick >= DC_CONTROL_PRESCALER) {
    dcControlTick = 0;
    if (motorPower > 0) {
      heart.motor0_reverse(turnmotor0*(motorPower + offsetMotorPower));
      heart.motor1_reverse(turnmotor1*(motorPower + offsetMotorPower));
    } else if (motorPower < 0) {
      heart.motor0_forward(turnmotor0*(-motorPower - offsetMotorPower));
      heart.motor1_forward(turnmotor1*(-motorPower - offsetMotorPower));
    }
  }
 
  // Activate safety timer every CONTROL_PERIOD_US*WD_TIMER_PRESCALER 
  if (wdTimerTick >= WD_TIMER_PRESCALER) {
    wdTimerTick = 0;
    reset_control();
  }

  if (!bleDeviceConnected && !bleConnUpdated) {
    // BLE disconnecting
    ble.StartServiceAdvertising();
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
