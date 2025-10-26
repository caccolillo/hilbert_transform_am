#include <ap_int.h>
#include <ap_fixed.h>
#include <math.h>


// Data format
const int DataWordSize = 25;
const int DataIntSize = 3;
const float DataMaxVal = pow(2.0, DataIntSize-1);
typedef ap_fixed<DataWordSize, DataIntSize, AP_RND, AP_SAT, 0> data_type;


//coefficient constants and data types
const int CoeffWordSize = 18;
const int CoeffIntSize = 4;
typedef ap_fixed<CoeffWordSize, CoeffIntSize, AP_RND, AP_SAT, 0> coeff_type;
const coeff_type scaleconst1 = 1.0980991655570851E-03;
const coeff_type coeff_b1_section1 = 1.0000000000000000E+00;
const coeff_type coeff_b2_section1 = 2.0000000000000000E+00;
const coeff_type coeff_b3_section1 = 1.0000000000000000E+00;
const coeff_type coeff_a2_section1 = -1.9041023073513077E+00;
const coeff_type coeff_a3_section1 = 9.0849470401353605E-01;


typedef ap_fixed<DataWordSize+CoeffWordSize+3, DataIntSize+CoeffIntSize, AP_TRN, AP_WRAP, 0> accum_type;

