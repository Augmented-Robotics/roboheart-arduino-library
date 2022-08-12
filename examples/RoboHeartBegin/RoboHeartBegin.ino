/* This example shows the minimal running
 * instance of the library.
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart();

void setup() {
    // Set up the RoboHeart
    heart.begin();
}

void loop() {
    // Give computing time to the RoboHeart
    heart.beat();
}