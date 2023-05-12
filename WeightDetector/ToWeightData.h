#ifndef TO_WEIGHT_DATA
#define TO_WEIGHT_DATA 6

#ifndef MY_DATA
#define MY_DATA
typedef int mydata;
#endif

#ifndef MY_DEMICAL
#define MY_DEMICAL
typedef float demical;
#endif

class ToWeightData{

public:
  static mydata analog2digit(demical deltaAngualr, demical deltax);
};

//mydata analog2digit(demical deltaAngular, demical deltax);

#endif
