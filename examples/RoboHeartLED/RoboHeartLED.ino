/* Hello World Example with the LED.
 * Toggel LED every half second. 
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart();
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