/* This example shows how to get information from
 * the built-in IMU.
 *
 * Created  30/01/2022
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>
#include <Wire.h>

unsigned long prevPrintTime = 0;

RoboHeart heart = RoboHeart(Serial);

void setup() {
    Serial.begin(115200);

    // Set up the RoboHeart
    heart.begin();

    Serial.println("RoboHeart MPU Demo");
}

void loop() {
   
    // print IMU data every second
    if (millis() - prevPrintTime > 1000) {
    Serial.print("\nAccelerometer [G]:\n");
    Serial.print(" X = ");
    Serial.println(heart.imu.readFloatAccelX(), 4);
    Serial.print(" Y = ");
    Serial.println(heart.imu.readFloatAccelY(), 4);
    Serial.print(" Z = ");
    Serial.println(heart.imu.readFloatAccelZ(), 4);

    Serial.print("\nGyroscope [angular velocity]:\n");
    Serial.print(" X = ");
    Serial.println(heart.imu.readFloatGyroX(), 4);
    Serial.print(" Y = ");
    Serial.println(heart.imu.readFloatGyroY(), 4);
    Serial.print(" Z = ");
    Serial.println(heart.imu.readFloatGyroZ(), 4);

    Serial.print("\nThermometer:\n");
    Serial.print(" Degrees C = ");
    Serial.println(heart.imu.readTempC(), 4);
    Serial.print(" Degrees F = ");
    Serial.println(heart.imu.readTempF(), 4);
    F("=====================================================\n");
    prevPrintTime = millis();
    }
}
