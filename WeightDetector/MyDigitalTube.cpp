#include "MyDigitalTube.h"

MyDigitalTube::MyDigitalTube(int latchPin, int colckPin, int dataPin)
{
    this->latchPin = latchPin;
    this->clockPin = clockPin;
    this->dataPin = dataPin;
    this->begin();
}

void MyDigitalTube::light(int i)
{
    if(i>9999) i  = 9999;
    if(i < 0) i = 0;
    unsigned char gewei = (i % 100) % 10;
    unsigned char shiwei = (i % 100) / 10;
    unsigned char baiwei = (i % 1000) / 100;
    unsigned char qianwei = i / 1000;
    digitalWrite(latchPin, LOW); //low voltage to start
    shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[gewei]);
    shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[shiwei]);
    shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[baiwei]);
    shiftOut(dataPin, clockPin, MSBFIRST, tubeNums[qianwei]);
    digitalWrite(latchPin, HIGH); //high voltage to over
    delayMicroseconds(2);
    digitalWrite(latchPin, HIGH);//ST_CP
}

void MyDigitalTube::begin()
{
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
}
