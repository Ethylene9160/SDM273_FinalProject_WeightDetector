#ifndef _MY_AVERAGE_FILTER_
#define _MY_AVERAGE_FILTER_ 3
#include <Math.h>
//#include<queue>
#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif



#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif
//using namespace std;
class AverageInteface {
public:
	virtual void show(mydata data) = 0;
};

class MyAverageFilter {
private:
	mydata length;
	mydata average;
	mydata absoluteError;
	int calculator;
	demical relativeError;
	AverageInteface* averageInteface;
	//<mydata> *my_queue;
	mydata* my_queue;
	bool isFull;
	bool isBalence;

	void calculateAverage();

	bool checkBalence();

	bool checkSingleData(mydata data);

	void addElement(mydata data);

	void reset();

  //mydata abs(mydata a); 
public:
	MyAverageFilter(AverageInteface* averageInteface);

	MyAverageFilter(AverageInteface* averageInteface, mydata length);

	MyAverageFilter(AverageInteface* averageInteface, mydata length, mydata absoluteError, demical relativeError);

	void update(mydata data);
};


#endif
