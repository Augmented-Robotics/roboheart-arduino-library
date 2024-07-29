/*!
 *  @file RoboHeartDRV8836.cpp
 *
 * 	Arduino library for the RoboHeart.
 *
 */

#include "RoboHeartDRV8836.h"

#define FILE_IDENTIFIER \
    "DRV8836"  // Define identifier before including DebuggerMsgs.h
#include "DebuggerMsgs.h"

int clampSpeed(int speed, int minSpeed, int maxSpeed) {
    if (speed < minSpeed) {
        return minSpeed;
    } else if (speed > maxSpeed) {
        return maxSpeed;
    } else {
        return speed;
    }
}

RoboHeartDRV8836::RoboHeartDRV8836() {}
RoboHeartDRV8836::RoboHeartDRV8836(Stream& debug) : _debug(&debug) {}

RoboHeartDRV8836::~RoboHeartDRV8836() {}

void RoboHeartDRV8836::begin(int in1Pin, int in2Pin, int nsleepPin,
                             int in1Channel, int in2Channel) {
    _nSleepPin = nsleepPin;

    _in1Pin = in1Pin;
    _in2Pin = in2Pin;

    _in1Channel = in1Channel;
    _in2Channel = in2Channel;

    // Configure the pins
    pinMode(_in1Pin, OUTPUT);
    pinMode(_in2Pin, OUTPUT);

    // Calling only once the channels are set
    configPWM();

    // IN/IN MODE BY DEFAULT, IT SUPPORTS COASTING
    pinMode(_nSleepPin, OUTPUT);  //  nSLEEP
    digitalWrite(_nSleepPin, HIGH);
}

void RoboHeartDRV8836::configPWM(int freq, int resolution) {
    RETURN_WARN_IF_EQUAL(_in1Pin, -1)

    _pwmFreq = freq;
    _pwmResolution = resolution;

    ledcAttachChannel(_in1Pin, _pwmFreq, _pwmResolution, _in1Channel);
    ledcAttachChannel(_in2Pin, _pwmFreq, _pwmResolution, _in2Channel);

    _pwmMaxDutyCycle = (int)(pow(2, _pwmResolution) - 1);
}

void RoboHeartDRV8836::sleep(bool sleep) {
    RETURN_WARN_IF_EQUAL(_in1Pin, -1)

    ledcWrite(_in1Pin, 0);
    ledcWrite(_in2Pin, 0);
    digitalWrite(_nSleepPin, !sleep);
}

void RoboHeartDRV8836::coast() {
    RETURN_WARN_IF_EQUAL(_in1Pin, -1)

    ledcWrite(_in1Pin, 0);
    ledcWrite(_in2Pin, 0);
}

void RoboHeartDRV8836::forward(int speed) {
    RETURN_WARN_IF_EQUAL(_in1Pin, -1)
    _speed = clampSpeed(speed, 0, _pwmMaxDutyCycle);
    ledcWrite(_in1Pin, _speed);
    ledcWrite(_in2Pin, 0);
}

void RoboHeartDRV8836::reverse(int speed) {
    RETURN_WARN_IF_EQUAL(_in1Pin, -1)
    _speed = clampSpeed(speed, 0, _pwmMaxDutyCycle);
    ledcWrite(_in1Pin, 0);
    ledcWrite(_in2Pin, _speed);
}

void RoboHeartDRV8836::brake() {
    RETURN_WARN_IF_EQUAL(_in1Pin, -1)

    ledcWrite(_in1Pin, _pwmMaxDutyCycle);
    ledcWrite(_in2Pin, _pwmMaxDutyCycle);
}

int RoboHeartDRV8836::getSpeed() { return _speed; }

int RoboHeartDRV8836::getMaxDutyCycle() { return _pwmMaxDutyCycle; }
