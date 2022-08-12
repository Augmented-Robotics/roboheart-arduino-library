/* This example shows how to control the built-in
 * DC motor drivers
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart();

void setup() {
    Serial.begin(115200);
    Serial.println("RoboHeart Motor Test");
    heart.begin();
    delay(1000);
}

void loop() {
    Serial.println("Motor0");
    heart.motor0.reverse(100);
    delay(5000);
    heart.motor0.coast();

    Serial.println("Motor1");
    heart.motor1.reverse(127);
    delay(5000);
    heart.motor1.coast();

    Serial.println("Motor2");
    heart.motor2.reverse(255);
    delay(5000);
    heart.motor2.coast();

    heart.beat();
}