#include <ap_int.h>
#include <ap_fixed.h>
#include <hls_math.h>


// Data format
const int DataWordSize = 25;
const int DataIntSize = 3;
typedef ap_fixed<DataWordSize, DataIntSize, AP_RND, AP_SAT, 0> data_type;
//typedef float data_type;

data_type envelope_detector(data_type x, data_type h);
