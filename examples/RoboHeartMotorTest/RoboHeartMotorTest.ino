/* This example shows how to control the built-in
 * DC motor drivers.
 *
 * Created  30/01/2022
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart(Serial);

void setup() {
    Serial.begin(115200);

    // Set up the RoboHeart
    heart.begin();

    Serial.println("RoboHeart DC Motor Demo");
}

void loop() {
    // Run motorA
    Serial.println("motorA");
    heart.motorA.forward(100);
    delay(2000);
    heart.motorA.coast();

    // Run motorB
    Serial.println("motorB");
    heart.motorB.forward(127);
    delay(2000);
    heart.motorB.coast();

    // Run motorC
    Serial.println("motorC");
    heart.motorC.forward(255);
    delay(2000);
    heart.motorC.coast();
}