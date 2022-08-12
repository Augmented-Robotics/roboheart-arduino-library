/* This example shows how to get information from
 * the built-in IMU.
 */

#include <RoboHeart.h>
#include <Wire.h>

unsigned long prevPrintTime = 0;

RoboHeart heart = RoboHeart();

void setup() {
    Serial.begin(115200);

    // Set up the RoboHeart
    heart.begin();

    Serial.println("RoboHeart MPU Demo");
}

void loop() {
    // Give computing time to the RoboHeart
    heart.beat();

    // print IMU data every second
    if (millis() - prevPrintTime > 1000) {
        Serial.print(F("TEMPERATURE: "));
        Serial.println(heart.mpu.getTemp());
        Serial.print(F("ACCELERO  X: "));
        Serial.print(heart.mpu.getAccX());
        Serial.print("\tY: ");
        Serial.print(heart.mpu.getAccY());
        Serial.print("\tZ: ");
        Serial.println(heart.mpu.getAccZ());

        Serial.print(F("GYRO      X: "));
        Serial.print(heart.mpu.getGyroX());
        Serial.print("\tY: ");
        Serial.print(heart.mpu.getGyroY());
        Serial.print("\tZ: ");
        Serial.println(heart.mpu.getGyroZ());

        Serial.print(F("ACC ANGLE X: "));
        Serial.print(heart.mpu.getAccAngleX());
        Serial.print("\tY: ");
        Serial.println(heart.mpu.getAccAngleY());

        Serial.print(F("ANGLE     X: "));
        Serial.print(heart.mpu.getAngleX());
        Serial.print("\tY: ");
        Serial.print(heart.mpu.getAngleY());
        Serial.print("\tZ: ");
        Serial.println(heart.mpu.getAngleZ());
        Serial.println(
            F("=====================================================\n"));
        prevPrintTime = millis();
    }
}