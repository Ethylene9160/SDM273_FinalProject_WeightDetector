#include "MyKinematicDetector.h"
void deleteAll(ListNode* node);
/**********class MyKinematicDetector*********/
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
	//?????
	for (int i = 0; i < nCalibTimes; ++i) {
		int mpuVals[nValCnt];
		readAccGyr(mpuVals);
		for (int j = 0; j < nValCnt; ++j) {
			valSums[j] += mpuVals[j];
		}
	}
	//???????
	for (int i = 0; i < nValCnt; ++i) {
		calibData[i] = int(valSums[i] / nCalibTimes);
	}
	calibData[2] += 16384; //??о?Z????????￡??趨?????????
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

demical MyKinematicDetector ::getPitch(demical* pRealVals, demical fNorm) {
  demical fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  demical fCos = fNormYZ / fNorm;
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



MyKinematicDetector::MyKinematicDetector()
{
	ListNode* l0 = new ListNode(0,nullptr);
	ListNode* l1 = new ListNode(0,l0);
	ListNode* l2 = new ListNode(0,l1);
	ListNode* l3 = new ListNode(0,l2);
	ListNode* l4 = new ListNode(0,l3);
	ListNode* l5 = new ListNode(0,l4);
	ListNode* l6 = new ListNode(0,l5);
	ListNode* l7 = new ListNode(0,l6);
	ListNode* l8 = new ListNode(0,l7);
	this->startNode = new ListNode(0,l8);
	//(*this).begin();
	//this->filterPower = {0.3,0.2,0.1,0.1,0.1,0.05,0.05,0.05,0.03,0.02}; 
  //this->myFilter = new MyAverageFilter(this, 10, 10, 0.05);
  cp_num += 11;
}

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
	Serial.print(fNewRoll); 
	//Serial.print('(');
	//Serial.print(fRollRate); 
	Serial.print("\nPitch:");
	Serial.print(fNewPitch); 
	//Serial.print('(');
	//Serial.print(fPitchRate); 
	Serial.print("\n");
	//delay(10);
}

DataStorager* MyKinematicDetector::beforeRead()
{
  int readouts[nValCnt];
  readAccGyr(readouts);

  demical realVals[7];
  rectify(readouts, realVals);
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  cp_num++;
  return new DataStorager(realVals, fNorm);
}

demical MyKinematicDetector::getRoll()
{
  DataStorager* storager = beforeRead();
  demical rv = *(storager->getVals());
  demical fNorm = storager->getNorm();
  cp_num--;
  delete storager;
  return this->getRoll(&rv, fNorm);
}

demical MyKinematicDetector::getPitch()
{
  /*
  DataStorager* storager = beforeRead();
  
  demical fPitch = this->getPitch(storager->getVals(), storager->getNorm());
  
  if (*(storager->getVals()) < 0) {
    fPitch = -fPitch;
  }
  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;
  
  //demical fNewPitch = kalmanPitch.getAngle(fPitch, storager->getVals()[5], dt);
  //demical fNewPitch = fPitch*filterCoeffecient + (1.f-filterCoeffecient)*fLastPitch;
  push(fPitch);
  pop();
  
  demical fNewPitch = calculateOutput();
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;
	myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
	//demical fPitchRate = (fNewPitch - fLastPitch) / dt;

  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  cp_num--;
  delete storager;
  return fNewPitch;
  */



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
  //push(fPitch);
  //pop();
  //demical fNewPitch = calculateOutput();
  
  demical fNewPitch = fPitch*filterCoeffecient + (1.f-filterCoeffecient)*fLastPitch;
  
  /*
  push(fPitch);
  pop();
  
  demical fNewPitch = calculateOutput();
  */
  //myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  return fNewPitch;
  
}

void MyKinematicDetector::push(demical val){
	ListNode* node = startNode;
	while(node->next != nullptr){
		node = node->next;
	}
	node->next = new ListNode(val);
	cp_num++;
}

void MyKinematicDetector::pop(){
	ListNode* node = startNode;
	startNode = startNode->next;
	cp_num--;
	delete node;
}

demical MyKinematicDetector::getYaw()
{
  /*
  DataStorager* storager = beforeRead();
  demical rv = *(storager->getVals());
  demical fNorm = storager->getNorm();
  cp_num--;
  delete storager;
  

  return this->getYaw(&rv, fNorm);
  */
  int readouts[nValCnt];
  readAccGyr(readouts); 

  demical realVals[7];
  rectify(readouts, realVals); 

  
  demical fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);

  demical fPitch = getYaw(realVals, fNorm); 
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }
  unsigned long nCurTime = micros();
  demical dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //push(fPitch);
  //pop();
  //demical fNewPitch = calculateOutput();
  
  demical fNewPitch = fPitch*filterCoeffecient + (1.f-filterCoeffecient)*fLastPitch;
  
  /*
  push(fPitch);
  pop();
  
  demical fNewPitch = calculateOutput();
  */
  //myFilter->update(ToWeightData::analog2digit(fNewPitch, 0));
  demical fPitchRate = (fNewPitch - fLastPitch) / dt;
  fLastPitch = fNewPitch;
  nLastTime = nCurTime;
  return fNewPitch;
}


demical MyKinematicDetector::getAngle(){
//  return this->getPitch();
  return this->getYaw();
}

demical MyKinematicDetector::calculateOutput(){
	ListNode* node = startNode;
	demical sum = 0;
	for(int i=0;i<10;++i){
		sum += (node->val) * filterPower[i];
		node = node->next;
	}
	return sum;
} 

void MyKinematicDetector::show(mydata d){
  Serial.print("myAverageFilter: ");
  Serial.println(d);
}

MyKinematicDetector::~MyKinematicDetector(){
	cp_num--;
	//delete this->myFilter;
	
	deleteAll(this->startNode);
} 
/********** end MyKinematic class ********/

/********* class MyKinematic class *******/
DataStorager::DataStorager(demical* realVals, demical fNorm)
{
  this->realVals = new demical[7];
  cp_num++;
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
	cp_num--;
	delete[] this->realVals;
}
/*********** end DataStorager class *********/

/********** struct ListNode *************/
ListNode::ListNode():ListNode(0){}
ListNode::ListNode(demical val):ListNode(0, nullptr){}
ListNode::ListNode(demical x, ListNode* node){
	val = x;
	next = node;
}
/*********** end ListNode ***************/
void deleteAll(ListNode* node){
	while(node != nullptr){
		ListNode* n = node;
		node = node->next;
		delete n;
		cp_num--;
	}
}
