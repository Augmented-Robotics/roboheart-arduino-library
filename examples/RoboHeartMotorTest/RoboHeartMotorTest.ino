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
    Serial.println("motorA");
    heart.motorA.reverse(100);
    delay(5000);
    heart.motorA.coast();

    Serial.println("motorB");
    heart.motorB.reverse(127);
    delay(5000);
    heart.motorB.coast();

    Serial.println("motorC");
    heart.motorC.reverse(255);
    delay(5000);
    heart.motorC.coast();

    heart.beat();
}