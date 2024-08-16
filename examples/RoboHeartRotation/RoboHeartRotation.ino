/* This example shows how to calculate rotation
 * from the built-in IMU data.
 *
 * Created  27/06/2024
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <Wire.h>
#include <math.h>
#define TRESHOLD 0.1  //treshold in rad/s

float drift_x = 0;
float drift_y = 0;
float drift_z = 0;

unsigned long prevSamplingTime = 0;
unsigned long prevPrintTime = 0;

RoboHeart heart = RoboHeart(Serial);

void calculateDrifts(int timeout_ms = 500) {
  unsigned long time = millis();
  for (int i = 0; i < timeout_ms; i++) {
    drift_x += heart.imu.readFloatGyroX() / timeout_ms;
    drift_y += heart.imu.readFloatGyroY() / timeout_ms;
    drift_z += heart.imu.readFloatGyroZ() / timeout_ms;
    delay(1);
  }
}

bool isCalibrated(int timeout_ms = 1000) {
  unsigned long time = millis();
  long counter = 0;
  while ((millis() - time) < timeout_ms) {
    if (abs(heart.imu.readFloatGyroX() - drift_x) > TRESHOLD || abs(heart.imu.readFloatGyroY() - drift_y) > TRESHOLD || abs(heart.imu.readFloatGyroZ() - drift_z) > TRESHOLD) {
      counter++;
    }
    delay(1);
  }
  return (counter < (timeout_ms / 5));
}

float rotation[3] = { 0, 0, 0 };
void setup() {
  Serial.begin(115200);

  // Set up the RoboHeart
  heart.begin();

  Serial.println("RoboHeart LSM6DS3 rotation demo");

  calculateDrifts();
  int counter = 0;
  Serial.println("RoboHeart calibrating...");
  Serial.println("Please let it in stable position...");
  while (isCalibrated() == 0) {
      counter++;
      if(counter > 3){
          calculateDrifts();
          counter = 0;
          Serial.println("Calibration failed, trying again...");
      }
  }  

  Serial.println("RoboHeart calibrated");
}

void loop() {
  unsigned long actualTime = millis();
  if (actualTime - prevSamplingTime > 10) {
    float a_v_x = heart.imu.readFloatGyroX() - drift_x;
    float a_v_y = heart.imu.readFloatGyroY() - drift_y;
    float a_v_z = heart.imu.readFloatGyroZ() - drift_z;
    rotation[0] += a_v_x * (actualTime - prevSamplingTime) * 0.001;
    rotation[1] += a_v_y * (actualTime - prevSamplingTime) * 0.001;
    rotation[2] += a_v_z * (actualTime - prevSamplingTime) * 0.001;

    if (rotation[0] > 360) {
      rotation[0] -= 360;
    } else if (rotation[0] < 0) {
      rotation[0] += 360;
    }
    if (rotation[1] > 360) {
      rotation[1] -= 360;
    } else if (rotation[1] < 0) {
      rotation[1] += 360;
    }
    if (rotation[2] > 360) {
      rotation[2] -= 360;
    } else if (rotation[2] < 0) {
      rotation[2] += 360;
    }

    prevSamplingTime = millis();
  }
  if (actualTime - prevPrintTime > 1000) {
    Serial.print("\nRotation:\n");
    Serial.print(" X = ");
    Serial.println(rotation[0], 4);
    Serial.print(" Y = ");
    Serial.println(rotation[1], 4);
    Serial.print(" Z = ");
    Serial.println(rotation[2], 4);

    prevPrintTime = millis();
  }
}
