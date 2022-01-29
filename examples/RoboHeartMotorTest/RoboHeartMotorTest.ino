#include <RoboHeart.h>

RoboHeart heart = RoboHeart();

void setup()
{
    heart.begin();
    Serial.begin(115200);
    Serial.println("RoboHeart Motor Test");
    delay(1000);
}

void loop()
{
    Serial.println("Motors COAST");
    heart.motor0_coast();
    heart.motor1_coast();
    heart.motor2_coast();
    delay(5000);
    Serial.println("Motors REVERSE");
    heart.motor0_reverse();
    heart.motor1_reverse();
    heart.motor2_reverse();
    delay(5000);
    Serial.println("Motors BREAK");
    heart.motor0_brake();
    heart.motor1_brake();
    heart.motor2_brake();
    delay(5000);
     Serial.println("Motors FORWARD");
    heart.motor0_forward();
    heart.motor1_forward();
    heart.motor2_forward();
    delay(5000);


    heart.beat();
}