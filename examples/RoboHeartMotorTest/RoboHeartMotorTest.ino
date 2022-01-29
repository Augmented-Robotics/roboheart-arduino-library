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
    Serial.println("Motor0 COAST");
    heart.motor0_coast();
    delay(5000);
    Serial.println("Motor0 REVERSE");
    heart.motor0_reverse();
    delay(5000);
    Serial.println("Motor0 BREAK");
    heart.motor0_brake();
    delay(5000);
     Serial.println("Motor0 FORWARD");
    heart.motor0_forward();
    delay(5000);


    heart.beat();
}