
#include "RoboHeartDRV8836.h"

#define DEBUG_DRV8836(x) {if (_debug != NULL) {_debug->print("[DRV8836_DEBUG] "); _debug->print(x);}}
#define DEBUG_LN_DRV8836(x) {if (_debug != NULL) {_debug->print("[DRV8836_DEBUG] "); _debug->println(x);}}
#define DEBUG(x) {if (_debug != NULL) {_debug->print(x);}}
#define DEBUG_LN(x) {if (_debug != NULL) {_debug->println(x);}}

int clampSpeed(int speed, int minSpeed, int maxSpeed)
{
    if (speed < minSpeed) 
    {
        return minSpeed;
    } 
    else if (speed > maxSpeed)
    {
        return maxSpeed;
    }
    else 
    {
        return speed;
    }

}
RoboHeartDRV8836::RoboHeartDRV8836()
{
    _debug = NULL;

}
RoboHeartDRV8836::RoboHeartDRV8836(Stream& debug)
: _debug(&debug)
{
   
}

RoboHeartDRV8836::~RoboHeartDRV8836()
{
   
}

void RoboHeartDRV8836::begin(int modePin, int in1Pin, int in2Pin, int nsleepPin, Motor_DRV_t mDRV)
{ 
    _modePin = modePin;
    _nsleepPin = nsleepPin;

    // PWM channel choice depends on the chosen Driver
    _in1Pin = in1Pin;
    _in1Channel = (PWM_Channel_t)(mDRV*2); 
    ledcAttachPin(_in1Pin, _in1Channel);

    _in2Pin = in2Pin;
    _in2Channel = (PWM_Channel_t)(mDRV*2 + 1);
    ledcAttachPin(_in2Pin, _in2Channel);
    
    // Calling only once the channels are set
    configPWM();

    // MODE PIN LOW --> IN/IN MODE
    // MODE PIN HIGH --> PHASE/ENABLE MODE
    // WE USE IN/IN MODE, BECAUSE IT SUPPORTS COASTING
    pinMode(_modePin, OUTPUT);  //  MODE
    digitalWrite(_modePin, LOW);
    pinMode(_nsleepPin, OUTPUT);    //  nSLEEP
    digitalWrite(_nsleepPin, HIGH);
}

void RoboHeartDRV8836::configPWM(int freq, int resolution) 
{
    _pwmFreq = freq;
    _pwmResolution = resolution;
    ledcSetup(_in1Channel, _pwmFreq, _pwmResolution);  
    ledcSetup(_in2Channel, _pwmFreq, _pwmResolution); 

    _pwmMaxDutyCycle =  (int)(pow(2, _pwmResolution) - 1);

}

 void RoboHeartDRV8836::sleep(bool sleep)
 {
    ledcWrite(_in1Channel, 0);
    ledcWrite(_in2Channel, 0);
    digitalWrite(_nsleepPin,!sleep);   
 }


 void RoboHeartDRV8836::coast()
 {
    ledcWrite(_in1Channel, 0);
    ledcWrite(_in2Channel, 0);
 }

void RoboHeartDRV8836::forward(int speed)
{
    DEBUG_DRV8836("Received speed: ");
    DEBUG_LN(speed);
    _speed = clampSpeed(speed, 0, _pwmMaxDutyCycle);
    DEBUG_DRV8836("Running at speed: ");
    DEBUG_LN(_speed);
    ledcWrite(_in2Channel, 0);
    ledcWrite(_in1Channel, _speed);
}

void RoboHeartDRV8836::reverse(int speed)
{
    DEBUG_DRV8836("Received speed: ");
    DEBUG_LN(speed);
    _speed = clampSpeed(speed, 0, _pwmMaxDutyCycle);
    DEBUG_DRV8836("Running at speed: ");
    DEBUG_LN(_speed);
    ledcWrite(_in1Channel, 0);
    ledcWrite(_in2Channel, _speed);
}

void RoboHeartDRV8836::brake()
{
    ledcWrite(_in1Channel, _pwmMaxDutyCycle);
    ledcWrite(_in2Channel, _pwmMaxDutyCycle);
}

int RoboHeartDRV8836::getSpeed()
{
    return _speed;
}

int RoboHeartDRV8836::getMaxDutyCycle()
{
    return _pwmMaxDutyCycle;
}

