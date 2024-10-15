/* This example shows how to set automatic rotation calculation
 * from the built-in IMU data.
 *
 * Created  15/10/2024
 * By Augmented Robotics
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart(Serial); 
 
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting RoboHeart automatic rotation example...");
  Serial.println("Please let RoboHeart in stable position...");
  heart.begin();
  heart.setAutomaticRotation();
}

void loop()
{
  Serial.print("\nRotation:\n");
  Serial.print(" X = ");
  Serial.print(heart.getRotationX(), 4);
  Serial.print(" Y = ");
  Serial.print(heart.getRotationY(), 4);
  Serial.print(" Z = ");
  Serial.print(heart.getRotationZ(), 4);
  Serial.println();
  delay(500);
}