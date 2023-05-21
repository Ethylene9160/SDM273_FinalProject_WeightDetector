//#include<arduino.h>
#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
  unsigned char tubeNums[] = {  //common positive
    B11000000, // 0
    B11111001, // 1
    B10100100, // 2
    B10110000, // 3
    B10011001, // 3
    B10010010, // 8
    B10000010, // 6
    B11111000, // 7
    B10000000, // 8
    B10010000 // 9
  };
int cp_num = 0;
const int LOAD = 5;
const int SCLK = 6;
const int SDI = 7;

const int LATCH_PIN = LOAD;
const int CLOCK_PIN = SCLK;
const int DATA_PIN = SDI;

#include"Kalman.h"
#include"MyAverageFilter.h"
#include"MyKinematicDetector.h"
#include"ToWeightData.h"
#include"DataInterface.h"
#include"MyDigitalTube.h"
//#include "MyDigitalTube.h"

void light(int i){
  if(i > 9999)i=9999;
  else if(i < 0) i = 0;
  unsigned char gewei = (i%100)%10;
    unsigned char shiwei = (i%100)/10;
    unsigned char baiwei = (i%1000)/100; 
    unsigned char qianwei = i/1000; 
      digitalWrite(LATCH_PIN,LOW); //低电位表示启动
      shiftOut(DATA_PIN,CLOCK_PIN,MSBFIRST,tubeNums[gewei]);
      shiftOut(DATA_PIN,CLOCK_PIN,MSBFIRST,tubeNums[shiwei]&0b01111111);
      shiftOut(DATA_PIN,CLOCK_PIN,MSBFIRST,tubeNums[baiwei]);
      shiftOut(DATA_PIN,CLOCK_PIN,MSBFIRST,tubeNums[qianwei]);
      digitalWrite(LATCH_PIN,HIGH); //高电位表示停止
      delayMicroseconds(2);
      digitalWrite(LATCH_PIN, HIGH);//ST_CP
      //delay(1000);
   
}

class Stablizer:public AverageInteface{
private:
    MyAverageFilter* filter;
    MyDigitalTube* digitalTube;
    void begin();
public:
    Stablizer(MyDigitalTube* digitalTube);
    Stablizer(int length, MyDigitalTube* digitalTube);
    Stablizer(int length, int absoluteError, int relativeError, MyDigitalTube* digitalTube);

    void update(mydata d);
    virtual void show(mydata data) override;
};

MyKinematicDetector* kinematicDetector;
MyDigitalTube* myTube = nullptr;
Stablizer* stablizer;

void setup(){
  kinematicDetector = new MyKinematicDetector();
  //myTube = new MyDigitalTube(LATCH_PIN,CLOCK_PIN,DATA_PIN);
  stablizer = new Stablizer(8, 25, 5, myTube);
  cp_num += 3;
  kinematicDetector->begin();
  //stablizer->begin();
  //myTube->begin();
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT); 
  Serial.begin(9600); 
}

void loop(){
  //light(2333);
  //myTube->light(2333);
  //delay(1000);
  
  //kinematicDetector->mainLoop();
  demical d = kinematicDetector->getAngle();
  Serial.print("afterFilter: ");
  Serial.println(ToWeightData::analog2digit(d, 0));
  stablizer->update(ToWeightData::analog2digit(d, 0));
  delay(20);
  //Serial.print("the number of current instances is: ");
  //Serial.print(cp_num);
  
}



Stablizer::Stablizer(MyDigitalTube* digitalTube):digitalTube(digitalTube) {
  filter = new MyAverageFilter(this);
  cp_num++;
  begin();
}
Stablizer::Stablizer(int length, MyDigitalTube* digitalTube):digitalTube(digitalTube){
  filter = new MyAverageFilter(this, length);
  cp_num++;
  begin();
}
Stablizer::Stablizer(int length, int absoluteError, int relativeError, MyDigitalTube* digitalTube):digitalTube(digitalTube){
  filter = new MyAverageFilter(this, length, absoluteError, relativeError);
  cp_num++;
  begin();
}
//override this method, and after you push a data in this stablizer, this method will be used.
void Stablizer::show(int data)
{
    Serial.print("FiltedWeight: ");
    Serial.println(data);
    //Serial.println(" g.");
//    this->digitalTube->light((int)data);
    light(data);
    //if(data > 9999) data = 9999;
    //if(data < 0) data = 0;
    //if(myTube != nullptr)myTube->light(data);
}

void Stablizer::update(mydata d) {
    filter->update(d);
}

void Stablizer::begin(){
  //myTube->begin();
  this->digitalTube->begin();
}
