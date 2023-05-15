#include "MyAverageFilter.h"
#include<arduino.h>

void MyAverageFilter::calculateAverage()
{
	average = 0;
	for (int i = 0; i < length; ++i) {
		average += *(my_queue + i);
	}
	average = average / length;
}

bool MyAverageFilter::checkBalence()
{
	if (isFull) {
		calculateAverage();
		for (int i = 0; i < length; ++i) {
			if (!checkSingleData(*(my_queue + i))) return false;
		}
	}
	return true;
}

bool MyAverageFilter::checkSingleData(mydata data)
{
	mydata absoluteError = abs(data-average);
	if (absoluteError > this->absoluteError) return false;
	if ((demical)absoluteError / (demical)average > relativeError) return false;
	return true;
}

void MyAverageFilter::addElement(mydata data)
{
	*(my_queue + calculator) = data;
	calculator = (++calculator) % length;
}

void MyAverageFilter::reset()
{
	isFull = false;
	isBalence = false;
	average = 0;
	calculator = 0;
	for (int i = 0; i < length; ++i) {
		*(my_queue + i) = 0;
	}
}

MyAverageFilter::MyAverageFilter(AverageInteface* averageInteface) :MyAverageFilter(averageInteface, _MY_AVERAGE_FILTER_)
{

}

MyAverageFilter::MyAverageFilter(AverageInteface* averageInteface, mydata length, mydata absoluteError, demical relativeError)
{
	//todo

	my_queue = new int[length];
	this->averageInteface = averageInteface;

	this->length = length;
	this->absoluteError = absoluteError;
	this->relativeError = relativeError;



	this->averageInteface->show(111);
	reset();
}

void MyAverageFilter::update(mydata data)
{
	if (isBalence) {
		if (checkSingleData(data)) {
			averageInteface->show(average);
			return;
		}
		else {
		}
		reset();
	}
	else {

		if (isFull) {
			if (checkBalence()) {
				isBalence = true;
			}
		}
		else {
			if (calculator + 1 == this->length) {
				isFull = true;
			}
		}
	}

	this->addElement(data);
	averageInteface->show(data);
}

MyAverageFilter::MyAverageFilter(AverageInteface* averageInteface, mydata length) :MyAverageFilter(averageInteface, length, 10, 0.05)
{

}

MyAverageFilter::~MyAverageFilter()
{
	delete[] my_queue;
}
//mydata MyAverageFilter::abs(mydata data){
//  return data > 0? data:-data;
//}
