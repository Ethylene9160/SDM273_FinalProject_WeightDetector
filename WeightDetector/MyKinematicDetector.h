//#pragma once
#ifndef MY_KINEMATICS_DETECTOR
#define MY_KINEMATICS_DETECTOR 3
#include<Arduino.h>
#include <Wire.h>
#include <Math.h>

#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

#include "Kalman.h"
#include "DataInterface.h"
#include "ToWeightData.h"
#include "MyAverageFilter.h"
//typedef int int16_t;//remenber to delete this!

class DataStorager{
private:
  demical* realVals;
  demical fNorm;
public:
  DataStorager(demical* realVals, demical fNorm);
  
  demical* getVals();

  demical getNorm();
	
  ~DataStorager();
};
class MyKinematicDetector:public AngleDetector, public AverageInteface
{
private:
	const demical fRad2Deg = 57.295779513f; //rad to degree
	static const int MPU = 0x68; //MPU-6050 I2C addressַ
	static const int nValCnt = 7; //the number of the register once read
  static const demical filterCoeffecient = 0.15;//coeffecient of 1 order filtering
  MyAverageFilter* myFilter;//delete this
  
	const int nCalibTimes = 1000; //clear time
	int calibData[nValCnt]; //clear data

	unsigned long nLastTime = 0; //last reading time
	demical fLastRoll = 0.0f; //last Roll
	demical fLastPitch = 0.0f; //last pitch
	Kalman kalmanRoll; //Roll filter
	Kalman kalmanPitch; //Pitch filter

  
  demical finalPitch;//remenber to detele this
	

	/*
	*��MPU6050д��һ���ֽڵ�����
	* ָ���Ĵ�����ַ��һ���ֽڵ�ֵ
	*/
	void writeMPUReg(int nReg, unsigned char nVal);

	/* ��MPU6050�������ٶȼ������������¶Ⱥ��������ٶȼ�
	* ������ָ����������
	*/
	void readAccGyr(int* pVals);

	/*�Դ�����������ͳ�ƣ�У׼ƽ��ƫ����*/
	void calibration();

	/*��MPU6050����һ���ֽڵ�����
	* ָ���Ĵ�����ַ�����ض�����ֵ
	*/
	unsigned char readMPUReg(int nReg);

	/*���Roll��*/
	demical getRoll(demical* pRealVals, demical fNorm);

	/*���Pitch��*/
	demical getPitch(demical* pRealVals, demical fNorm);

	/*���Yaw��*/
	demical getYaw(demical* pRealVals, demical fNorm);

	/*�Զ������о���������ƫ�ƣ���ת��Ϊ��������*/
	void rectify(int* pReadout, demical* pRealVals);
  
  DataStorager* beforeRead();
public:
	
  /*
  a init function for initializing this class, with MPU8265 inside.
  Do not use it.
  */
  void begin();
  /*
	constructor.
	Ples initialize it in <code>void setup()</code> function,
	use keyword <code>new</code> to establish this in the stack space, 
	will be a better choice.
	*/
	MyKinematicDetector();

	void mainLoop();
  
  demical getRoll();

  demical getPitch();

  demical getYaw();

  virtual demical getAngle();

  virtual void show(mydata data);
	
	~MyKinematicDetector();
};


#endif
