#include <RoboHeart.h>

RoboHeart heart = RoboHeart(Serial); 
 
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting RoboHeart automatic rotation example..");
  heart.begin();
  heart.setAutomaticRotation();
}

void loop()
{
    Serial.print("\nRotation:\n");
    Serial.print(" X = ");
    Serial.println(heart.getRotationX(), 4);
    Serial.print(" Y = ");
    Serial.println(heart.getRotationY(), 4);
    Serial.print(" Z = ");
    Serial.println(heart.getRotationZ(), 4);
    delay(2000);
}