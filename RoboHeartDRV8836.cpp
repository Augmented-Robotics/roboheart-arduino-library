
#include "RoboHeartDRV8836.h"

RoboHeartDRV8836::RoboHeartDRV8836()
{
   
}

RoboHeartDRV8836::~RoboHeartDRV8836()
{
   
}

 void RoboHeartDRV8836::begin(int modePin, int in1Pin, int in2Pin, int nsleepPin)
 {
    _modePin = modePin;
    _in1Pin = in1Pin;
    _in2Pin = in2Pin;
    _nsleepPin = nsleepPin;
    pinMode(_in1Pin, OUTPUT);//  PHASE/IN1
    digitalWrite(_in1Pin,LOW);
    pinMode(_in2Pin, OUTPUT);//  ENABLE/IN2
    digitalWrite(_in2Pin,LOW); 
    pinMode(_modePin, OUTPUT);//  MODE
    // MODE PIN LOW --> IN/IN MODE
    // MODE PIN HIGH --> PHASE/ENABLE MODE
    // WE USE IN/IN MODE, BECAUSE IT SUPPORTS COASTING
    digitalWrite(_modePin,LOW);
    pinMode(_nsleepPin, OUTPUT);//  nSLEEP
    digitalWrite(_nsleepPin,HIGH);
 }

 void RoboHeartDRV8836::coast()
 {
    analogWrite(_in1Pin, 0);
    analogWrite(_in2Pin, 0);
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, LOW);
    
 }

void RoboHeartDRV8836::forward(uint8_t speed)
{
    digitalWrite(_in2Pin, LOW);
     _speed = speed;
    
    if(speed == 255)
    {
        digitalWrite(_in1Pin, HIGH);
        return;
    }
    analogWrite(_in1Pin, speed);
}

void RoboHeartDRV8836::reverse(uint8_t speed)
{
    digitalWrite(_in1Pin, LOW);
    _speed = speed;
    
    if(speed == 255)
    {
        digitalWrite(_in2Pin, HIGH);
        return;
    }
    analogWrite(_in2Pin, speed);
}

void RoboHeartDRV8836::brake()
{
    digitalWrite(_in1Pin, HIGH);
    digitalWrite(_in2Pin, HIGH);
    _speed = 0;    
}

uint8_t RoboHeartDRV8836::getSpeed()
{
    return _speed;
}
