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
    heart.motor0_reverse(100);
    delay(5000);
    heart.motor0_coast();

    Serial.println("Motor1");
    heart.motor1_reverse(127);
    delay(5000);
    heart.motor1_coast();

    Serial.println("Motor2");
    heart.motor2_reverse(255);
    delay(5000);
    heart.motor2_coast();

    heart.beat();
}