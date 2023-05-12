#include "ToWeightData.h"

mydata ToWeightData::analog2digit(demical deltaAngular, demical deltax){
  return (mydata)(deltaAngular * 100.f);
}

//mydata analog2digit(demical deltaAngular, demical deltax)
