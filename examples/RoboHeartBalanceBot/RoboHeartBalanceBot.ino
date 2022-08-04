#include <RoboHeart.h>
#include <RoboHeartTimer.h>

RoboHeart heart = RoboHeart();

#define Kp  20
#define Kd  0.001
#define Ki  40

#define BUTTON_PIN 0

#define CONTROL_PERIOD_US   100.0

#define PID_CONTROL_PRESCALER   5   
#define DC_CONTROL_PRESCALER 15
#define STATISTICS_PRESCALER DC_CONTROL_PRESCALER*4

float prevAngleDeg = 0;
float motorPower = 0;
float errorSum = 0;
float currentAngleDeg = 0;
float targetAngleDeg = 0.;
float offsetAngleDeg = 0.;

unsigned long pidControlTick = 0;
unsigned long dcControlTick = 0;
unsigned long statisticsTick = 0;

unsigned long prevTimeIntervalMS = 0;

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
}

void processPinInterrupt()
{
  offsetAngleDeg = heart.getAngleX();
  targetAngleDeg = offsetAngleDeg;
}

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
      heart.motor0_reverse(motorPower);
      heart.motor1_reverse(motorPower);
    } else if (motorPower < 0) {
      heart.motor0_forward(-motorPower);
      heart.motor1_forward(-motorPower);
    }
  }

  // Print statistics every CONTROL_PERIOD_US*STATISTICS_PRESCALER 
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
