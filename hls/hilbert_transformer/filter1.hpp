#include <ap_int.h>
#include <ap_fixed.h>
#include <math.h>

#define FILTER_LENGTH 32
 
// Data format
const int DataWordSize = 25;
const int DataIntSize = 3;
const float DataMaxVal = pow(2.0, DataIntSize-1);
typedef ap_fixed<DataWordSize, DataIntSize, AP_RND, AP_SAT, 0> data_type;



//coefficient constants and data types
const int CoeffWordSize = 18;
const int CoeffIntSize = 4;
typedef ap_fixed<CoeffWordSize, CoeffIntSize, AP_RND, AP_SAT, 0> coeff_type;


// Filter coefficients
static const coeff_type coeffs[FILTER_LENGTH] = {
    -3.0838958070072532E-03,  // coeff1
    -1.1563322168801279E-03,  // coeff2
    -5.9708316939980901E-03,  // coeff3
    -3.3653501813806982E-03,  // coeff4
    -1.1370294595527887E-02,  // coeff5
    -7.5683440589330679E-03,  // coeff6
    -1.9628515643984545E-02,  // coeff7
    -1.4978186105303284E-02,  // coeff8
    -3.2113551266464938E-02,  // coeff9
    -2.7894786623769277E-02,  // coeff10
    -5.2056692551864792E-02,  // coeff11
    -5.2256610273355014E-02,  // coeff12
    -9.0302534572430176E-02,  // coeff13
    -1.1258848799869187E-01,  // coeff14
    -2.1700314947952859E-01,  // coeff15
    -6.2655534028311499E-01,  // coeff16
     6.2655534028311499E-01,  // coeff17
     2.1700314947952859E-01,  // coeff18
     1.1258848799869187E-01,  // coeff19
     9.0302534572430176E-02,  // coeff20
     5.2256610273355014E-02,  // coeff21
     5.2056692551864792E-02,  // coeff22
     2.7894786623769277E-02,  // coeff23
     3.2113551266464938E-02,  // coeff24
     1.4978186105303284E-02,  // coeff25
     1.9628515643984545E-02,  // coeff26
     7.5683440589330679E-03,  // coeff27
     1.1370294595527887E-02,  // coeff28
     3.3653501813806982E-03,  // coeff29
     5.9708316939980901E-03,  // coeff30
     1.1563322168801279E-03,  // coeff31
     3.0838958070072532E-03   // coeff32
};


// Coefficients for section 1
const coeff_type scaleconst1_section1 = 6.1170672733034965E-03;
const coeff_type coeff_a2_section1 = -1.9812743944956361E+00;
const coeff_type coeff_a3_section1 = 9.9095325489279062E-01;
const coeff_type scaleconst2_section1 = 6.1170672733034965E-03;

// Coefficients for section 2
const coeff_type coeff_a2_section2 = -1.9836937929369820E+00;
const coeff_type coeff_a3_section2 = 9.9174538760882180E-01;

typedef ap_fixed<DataWordSize+CoeffWordSize+3, DataIntSize+CoeffIntSize, AP_TRN, AP_WRAP, 0> accum_type;



// Processes one sample through the fixed-point IIR filter
data_type filter1(data_type input);
data_type section1(data_type input);
data_type section2(data_type input);


