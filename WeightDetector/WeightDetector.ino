//#include<arduino.h>
#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif

#include"Kalman.h"
#include"MyAverageFilter.h"
#include"MyKinematicDetector.h"
#include"ToWeightData.h"
#include"DataInterface.h"

class Stablizer:public AverageInteface{
private:
    MyAverageFilter* filter;
public:
    Stablizer();
    Stablizer(int length);
    Stablizer(int length, int absoluteError, int relativeError);

    void update(mydata d);
    virtual void show(mydata data) override;
};

MyKinematicDetector* kinematicDetector;



void setup(){
  Serial.begin(9600); 
  kinematicDetector = new MyKinematicDetector();
  //kinematicDetector->begin();
}

void loop(){
  //kinematicDetector->mainLoop();
  Serial.print("afterFilter: ");
  demical d = kinematicDetector->getAngle();
  Serial.println(ToWeightData::analog2digit(d, 0));
  delay(20);
}

Stablizer::Stablizer() {
  filter = new MyAverageFilter(this);
}
Stablizer::Stablizer(int length){
  filter = new MyAverageFilter(this, length);
}
Stablizer::Stablizer(int length, int absoluteError, int relativeError){
  filter = new MyAverageFilter(this, length, absoluteError, relativeError);
}
//override this method, and after you push a data in this stablizer, this method will be used.
void Stablizer::show(int data)
{
    Serial.print("Weight of which is: ");
    Serial.print(data);
    Serial.println(" g.");
}

void Stablizer::update(mydata d) {
    filter->update(d);
}
