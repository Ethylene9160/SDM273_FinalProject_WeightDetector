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
//#include"Main.cpp"

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
/*************************/
  const demical fRad2Deg = 57.295779513f; //rad to degree
  static const int MPU = 0x68; //MPU-6050 I2C addressַ
  static const int nValCnt = 7; //the number of the register once read

  const int nCalibTimes = 1000; //clear time
  int calibData[nValCnt]; //clear data

  unsigned long nLastTime = 0; //last reading time
  demical fLastRoll = 0.0f; //last Roll
  demical fLastPitch = 0.0f; //last pitch
  Kalman kalmanRoll; //Roll filter
  Kalman kalmanPitch; //Pitch filter
/****************************/


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





/*****************************************************/


void writeMPUReg(int nReg, unsigned char nVal)
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

unsigned char readMPUReg(int nReg)
{
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

void readAccGyr(int* pVals)
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

void calibration()
{
  demical valSums[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };

  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    readAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; 
}

demical getRoll(demical* pRealVals, demical fNorm)
{
  demical fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  demical fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

demical getYaw(demical* pRealVals, demical fNorm)
{
  demical fNormXY = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[1] * pRealVals[1]);
  demical fCos = fNormXY / fNorm;
  return acos(fCos) * fRad2Deg;
}

void rectify(int* pReadout, demical* pRealVals)
{
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (demical)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (demical)(pReadout[i] - calibData[i]) / 131.0f;
  }
}
void mainLoop()
{
  int readouts[nValCnt];
  readAccGyr(readouts); 

  demical realVals[7];
  rectify(readouts, realVals); 

  
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  Serial.print("fNorm: ");
  Serial.println(fNorm);
  demical fRoll = getRoll(realVals, fNorm); 
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  demical fPitch = getPitch(realVals, fNorm); 
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }

  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;
  
  demical fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  demical fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  
  demical fRollRate = (fNewRoll - fLastRoll) / dt;
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;

  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;

  Serial.print("Roll:");
  Serial.print(fNewRoll); Serial.print('(');
  Serial.print(fRollRate); Serial.print("),\tPitch:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(fPitchRate); Serial.print(")\n");
  //delay(10);
}

DataStorager* beforeRead()
{
  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  return new DataStorager(realVals, fNorm);
}

demical getPitch(demical* pRealVals, demical fNorm) {
  demical fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  demical fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

/*****************************************************/
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
