/* Hello World Example with the LED.
 * Toggel LED every half second. 
 * 
 * Created  30/01/2022
 * By Augmented Robotics 
 * 
 * Check out https://roboheart.de/en_gb/ for more information about RoboHeart.
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart(Serial);
bool ledState = HIGH; // Save LED state

void setup() {
    Serial.begin(115200);
    
    // Set up the RoboHeart
    heart.begin();

    // Configure the LED pin
    pinMode(LED_ROBOHEART, OUTPUT);

    Serial.println("RoboHeart LED Demo");
}

// Change LED state
void toggleLED() {
    if (ledState == HIGH) {
        Serial.println("LED status: Low");
        digitalWrite(LED_ROBOHEART, LOW);
        ledState = LOW;
    } else {
        Serial.println("LED status: High");
        digitalWrite(LED_ROBOHEART, HIGH);
        ledState = HIGH;
    }
}

void loop() {
    // Give computing time to the RoboHeart
    heart.beat();
    
    // LED indicator
    toggleLED();
    delay(500);

}