/* Hello World Example with the LED.
 * Toggel LED every half second. 
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart();
bool ledState = HIGH;

void setup() {
    Serial.begin(115200);
    
    // set up the RoboHeart
    heart.begin();

    // configure LED pin
    pinMode(LED_ROBOHEART, OUTPUT);
}

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
    // give computing time to the RoboHeart
    heart.beat();
    
    // LED indicator
    toggleLED();
    delay(500);

}