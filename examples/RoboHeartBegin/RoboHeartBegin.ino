/* This example shows the minimal running
 * instance of the library.
 */

#include <RoboHeart.h>

RoboHeart heart = RoboHeart();

void setup() {
    // set up the RoboHeart
    heart.begin();
}

void loop() {
    // give computing time to the RoboHeart
    heart.beat();
}