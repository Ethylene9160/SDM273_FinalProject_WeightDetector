#ifndef MY_DIGITAL_TUBE
#include<arduino.h>
#define MY_DIGITAL_TUBE 4

class MyDigitalTube {
private:
	int latchPin, clockPin, dataPin;

	void begin();
public:
	const static unsigned char tubeNums[10] = {  //common positive
  B11000000, // 0
  B11111001, // 1
  B10100100, // 2
  B10110000, // 3
  B10011001, // 3
  B10010010, // 8
  B10000010, // 6
  B11111000, // 7
  B10000000, // 8
  B10010000, // 9
	};

	MyDigitalTube(int latchPin, int colckPin, int dataPin);

	void light(int i);
};


#endif
