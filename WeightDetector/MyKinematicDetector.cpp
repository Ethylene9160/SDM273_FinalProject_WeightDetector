#include "MyKinematicDetector.h"

void MyKinematicDetector::begin()
{
	//Serial.begin(9600); 
	Wire.begin(); 
	writeMPUReg(0x6B, 0); 

	calibration();
	nLastTime = micros(); //record current time 
}

void MyKinematicDetector::writeMPUReg(int nReg, unsigned char nVal)
{
	Wire.beginTransmission(MPU);
	Wire.write(nReg);
	Wire.write(nVal);
	Wire.endTransmission(true);
}

unsigned char MyKinematicDetector::readMPUReg(int nReg)
{
	Wire.beginTransmission(MPU);
	Wire.write(nReg);
	Wire.requestFrom(MPU, 1, true);
	Wire.endTransmission(true);
	return Wire.read();
}

void MyKinematicDetector::readAccGyr(int* pVals)
{
	Wire.beginTransmission(MPU);
	Wire.write(0x3B);
	Wire.requestFrom(MPU, nValCnt * 2, true);
	Wire.endTransmission(true);
	for (long i = 0; i < nValCnt; ++i) {
		pVals[i] = Wire.read() << 8 | Wire.read();
	}
}

void MyKinematicDetector::calibration()
{
	demical valSums[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0 };
	//�����
	for (int i = 0; i < nCalibTimes; ++i) {
		int mpuVals[nValCnt];
		readAccGyr(mpuVals);
		for (int j = 0; j < nValCnt; ++j) {
			valSums[j] += mpuVals[j];
		}
	}
	//����ƽ��
	for (int i = 0; i < nValCnt; ++i) {
		calibData[i] = int(valSums[i] / nCalibTimes);
	}
	calibData[2] += 16384; //��оƬZ����ֱ���£��趨��̬�����㡣
}

demical MyKinematicDetector::getRoll(demical* pRealVals, demical fNorm)
{
	demical fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
	demical fCos = fNormXZ / fNorm;
	return acos(fCos) * fRad2Deg;
}

demical MyKinematicDetector::getYaw(demical* pRealVals, demical fNorm)
{
	demical fNormXY = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[1] * pRealVals[1]);
	demical fCos = fNormXY / fNorm;
	return acos(fCos) * fRad2Deg;
}

void MyKinematicDetector::rectify(int* pReadout, demical* pRealVals)
{
	for (int i = 0; i < 3; ++i) {
		pRealVals[i] = (demical)(pReadout[i] - calibData[i]) / 16384.0f;
	}
	pRealVals[3] = pReadout[3] / 340.0f + 36.53;
	for (int i = 4; i < 7; ++i) {
		pRealVals[i] = (demical)(pReadout[i] - calibData[i]) / 131.0f;
	}
}

demical MyKinematicDetector ::getPitch(demical* pRealVals, demical fNorm) {
	demical fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
	demical fCos = fNormYZ / fNorm;
	return acos(fCos) * fRad2Deg;
}



MyKinematicDetector::MyKinematicDetector()
{
	//(*this).begin();
  this->myFilter = new MyAverageFilter(this, 10, 10, 0.05);
}

//this is just a function to test whether the MPU6050 works well.
void MyKinematicDetector::mainLoop()
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

DataStorager* MyKinematicDetector::beforeRead()
{
  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  return new DataStorager(realVals, fNorm);
}

demical MyKinematicDetector::getRoll()
{
  DataStorager* storager = beforeRead();
  demical rv = *(storager->getVals());
  demical fNorm = storager->getNorm();
  delete storager;
  return this->getRoll(&rv, fNorm);
}

demical MyKinematicDetector::getPitch()
{
	DataStorager* storager = beforeRead();
  
 	demical fPitch = this->getPitch(storager->getVals(), storager->getNorm());
  
 	if (*(storager->getVals()) < 0) {
		fPitch = -fPitch;
 	}
 	unsigned long nCurTime = micros();
 	demical dt = (double)(nCurTime - nLastTime) / 1000000.0;
  
	demical fPitchRate = (fNewPitch - fLastPitch) / dt;
  
 	demical fNewPitch = fPitch*filterCoeffecient + (1.f-filterCoeffecient)*fLastPitch;
	myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
	demical fPitchRate = (fNewPitch - fLastPitch) / dt;

 	fLastPitch = fNewPitch;
 	nLastTime = nCurTime;
  
 	delete storager;
 	return fNewPitch;
  


/*
  int readouts[nValCnt];
  readAccGyr(readouts); 

  demical realVals[7];
  rectify(readouts, realVals); 

  
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);

  demical fPitch = getPitch(realVals, fNorm); 
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;
  
  demical fNewPitch = fPitch*filterCoeffecient + (1.f-filterCoeffecient)*fLastPitch;
  myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  return fNewPitch;
  */
}

demical MyKinematicDetector::getYaw()
{
	DataStorager* storager = beforeRead();
	demical rv = *(storager->getVals());
	demical fNorm = storager->getNorm();
	delete storager;
	

	return this->getYaw(&rv, fNorm);
}

DataStorager::DataStorager(demical* realVals, demical fNorm)
{
 	this->realVals = new demical[7];
 	for(int i = 0; i < 7;i++){
 		this->realVals[i] = realVals[i];
 	}
 	this->fNorm = fNorm;
}

demical* DataStorager::getVals()
{
 	return this->realVals;
}

demical DataStorager::getNorm()
{
 	return this->fNorm;
}

DataStorager:: ~DataStorager(){
	delete this->realVals;
}

demical MyKinematicDetector::getAngle(){
 	return this->getPitch();
}

void MyKinematicDetector::show(mydata d){
 	Serial.print("my_average_filter: ");
 	Serial.println(d);
}

MyAverageFilter::~MyAverageFilter()
{
	delete[] my_queue;
}
