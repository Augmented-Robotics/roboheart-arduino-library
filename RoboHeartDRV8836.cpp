/*!
 *  @file RoboHeartDRV8836.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeartDRV8836.h"

#define DEBUG_DRV8836(x)                       \
    {                                          \
        if (_debug != NULL) {                  \
            _debug->print("[DRV8836_DEBUG] "); \
            _debug->print(x);                  \
        }                                      \
    }
#define DEBUG_LN_DRV8836(x)                    \
    {                                          \
        if (_debug != NULL) {                  \
            _debug->print("[DRV8836_DEBUG] "); \
            _debug->println(x);                \
        }                                      \
    }
#define DEBUG(x)              \
    {                         \
        if (_debug != NULL) { \
            _debug->print(x); \
        }                     \
    }
#define DEBUG_LN(x)             \
    {                           \
        if (_debug != NULL) {   \
            _debug->println(x); \
        }                       \
    }

int clampSpeed(int speed, int minSpeed, int maxSpeed) {
    if (speed < minSpeed) {
        return minSpeed;
    } else if (speed > maxSpeed) {
        return maxSpeed;
    } else {
        return speed;
    }
}
RoboHeartDRV8836::RoboHeartDRV8836() { _debug = NULL; }
RoboHeartDRV8836::RoboHeartDRV8836(Stream& debug) : _debug(&debug) {}

RoboHeartDRV8836::~RoboHeartDRV8836() {}

void RoboHeartDRV8836::begin(int modePin, int in1Pin, int in2Pin,
                             int nsleepPin) {
    _modePin = modePin;
    _nsleepPin = nsleepPin;

    _in1Pin = in1Pin;
    _in2Pin = in2Pin;

    // Configure the pins
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);

    // Ensure that the PWM channel has been assigned to each pin
    analogWrite(_in1Pin, 0);
    analogWrite(_in2Pin, 0);

    // Calling only once the channels are set
    configPWM();

    // MODE PIN LOW --> IN/IN MODE
    // MODE PIN HIGH --> PHASE/ENABLE MODE
    // WE USE IN/IN MODE, BECAUSE IT SUPPORTS COASTING
    pinMode(_modePin, OUTPUT);  //  MODE
    digitalWrite(_modePin, LOW);
    pinMode(_nsleepPin, OUTPUT);  //  nSLEEP
    digitalWrite(_nsleepPin, HIGH);
}

void RoboHeartDRV8836::configPWM(int freq, int resolution) {
    _pwmFreq = freq;
    _pwmResolution = resolution;

    ledcSetup(analogGetChannel(_in1Pin), _pwmFreq, _pwmResolution);
    ledcSetup(analogGetChannel(_in2Pin), _pwmFreq, _pwmResolution);

    _pwmMaxDutyCycle = (int)(pow(2, _pwmResolution) - 1);
}

void RoboHeartDRV8836::sleep(bool sleep) {
    ledcWrite(analogGetChannel(_in1Pin), 0);
    ledcWrite(analogGetChannel(_in2Pin), 0);
    digitalWrite(_nsleepPin, !sleep);
}

void RoboHeartDRV8836::coast() {
    ledcWrite(analogGetChannel(_in1Pin), 0);
    ledcWrite(analogGetChannel(_in2Pin), 0);
}

void RoboHeartDRV8836::forward(int speed) {
    DEBUG_DRV8836("Received speed: ");
    DEBUG_LN(speed);
    _speed = clampSpeed(speed, 0, _pwmMaxDutyCycle);
    DEBUG_DRV8836("Running at speed: ");
    DEBUG_LN(_speed);
    ledcWrite(analogGetChannel(_in1Pin), _speed);
    ledcWrite(analogGetChannel(_in2Pin), 0);
}

void RoboHeartDRV8836::reverse(int speed) {
    DEBUG_DRV8836("Received speed: ");
    DEBUG_LN(speed);
    _speed = clampSpeed(speed, 0, _pwmMaxDutyCycle);
    DEBUG_DRV8836("Running at speed: ");
    DEBUG_LN(_speed);
    ledcWrite(analogGetChannel(_in1Pin), 0);
    ledcWrite(analogGetChannel(_in2Pin), _speed);
}

void RoboHeartDRV8836::brake() {
    ledcWrite(analogGetChannel(_in1Pin), _pwmMaxDutyCycle);
    ledcWrite(analogGetChannel(_in2Pin), _pwmMaxDutyCycle);
}

int RoboHeartDRV8836::getSpeed() { return _speed; }

int RoboHeartDRV8836::getMaxDutyCycle() { return _pwmMaxDutyCycle; }
