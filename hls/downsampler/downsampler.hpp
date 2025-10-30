#include <ap_int.h>
#include <ap_fixed.h>
#include <math.h>


// Data format
const int DataWordSize = 25;
const int DataIntSize = 3;
const float DataMaxVal = pow(2.0, DataIntSize-1);
typedef ap_fixed<DataWordSize, DataIntSize, AP_RND, AP_SAT, 0> data_type;

//Function prototype
data_type downsampler(data_type filter_in);
