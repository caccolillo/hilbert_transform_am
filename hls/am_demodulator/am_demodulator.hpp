#ifndef AM_DEMODULATOR_HPP
#define AM_DEMODULATOR_HPP

//#define DEBUG


#include <ap_int.h>
#include <ap_fixed.h>
#include <math.h>
#include "hls_stream.h"
#include "ap_axi_sdata.h"

#define FILTER_LENGTH 32

// Data format
const int DataWordSize = 40;
const int DataIntSize = 16;
const float DataMaxVal = pow(2.0, DataIntSize-1);

#ifdef DEBUG
  typedef float data_type;
#else
  typedef ap_fixed<DataWordSize, DataIntSize, AP_RND, AP_SAT, 0> data_type;
#endif


// Coefficient constants and data types
const int CoeffWordSize = 18;
const int CoeffIntSize = 4;

#ifdef DEBUG
  typedef float coeff_type;
#else
  typedef ap_fixed<CoeffWordSize, CoeffIntSize, AP_RND, AP_SAT, 0> coeff_type;
#endif


// Accumulator data types
#ifdef DEBUG
  typedef float accum_type;
#else
  typedef ap_fixed<DataWordSize+CoeffWordSize+20, DataIntSize+CoeffIntSize, AP_TRN, AP_WRAP, 0> accum_type;
#endif


// AXI Stream data type


typedef ap_axis<DataWordSize, 0, 0, 0> axis_data;

// IIR Filter coefficients
const coeff_type scaleconst1 = 1.0980991655570851E-03;
const coeff_type coeff_b1_section1 = 1.0000000000000000E+00;
const coeff_type coeff_b2_section1 = 2.0000000000000000E+00;
const coeff_type coeff_b3_section1 = 1.0000000000000000E+00;
const coeff_type coeff_a2_section1 = -1.9041023073513077E+00;
const coeff_type coeff_a3_section1 = 9.0849470401353605E-01;

// Hilbert filter coefficients
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

// Function prototypes
data_type mean(data_type filter_in);
data_type delay_line(data_type filter_in);
data_type downsampler(data_type filter_in);
data_type envelope_detector(data_type x, data_type h);
data_type applyIIRFilter(data_type filter_in);
data_type filter1(data_type input);

// Top-level function with AXI Stream interfaces
void am_demodulator(hls::stream<axis_data> &input_stream,
                    hls::stream<axis_data> &output_stream);

#endif // AM_DEMODULATOR_HPP
