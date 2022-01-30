#include <Wire.h>
#include <RoboHeart.h>

unsigned long timer = 0;

RoboHeart heart = RoboHeart();

void setup()
{
    Serial.begin(115200);
    Serial.println("RoboHeart MPU Demo");
    //set up the RoboHeart
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    delay(100);
    heart.begin();

}

void loop()
{
    //give computing time to the RoboHeart
    heart.beat();

    

  if(millis() - timer > 1000){ // print data every second
    Serial.print(F("TEMPERATURE: "));Serial.println(heart.getTemp());
    Serial.print(F("ACCELERO  X: "));Serial.print(heart.getAccX());
    Serial.print("\tY: ");Serial.print(heart.getAccY());
    Serial.print("\tZ: ");Serial.println(heart.getAccZ());
  
    Serial.print(F("GYRO      X: "));Serial.print(heart.getGyroX());
    Serial.print("\tY: ");Serial.print(heart.getGyroY());
    Serial.print("\tZ: ");Serial.println(heart.getGyroZ());
  
    Serial.print(F("ACC ANGLE X: "));Serial.print(heart.getAccAngleX());
    Serial.print("\tY: ");Serial.println(heart.getAccAngleY());
    
    Serial.print(F("ANGLE     X: "));Serial.print(heart.getAngleX());
    Serial.print("\tY: ");Serial.print(heart.getAngleY());
    Serial.print("\tZ: ");Serial.println(heart.getAngleZ());
    Serial.println(F("=====================================================\n"));
    timer = millis();
  }
  
  
}