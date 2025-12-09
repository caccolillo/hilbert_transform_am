
// ============================================================================
// downsampler.hpp
// ============================================================================
#ifndef DOWNSAMPLER_HPP
#define DOWNSAMPLER_HPP

#include <ap_int.h>
#include <ap_fixed.h>
#include <ap_axi_sdata.h>
#include <math.h>
#include "hls_stream.h"

#define DOWNSAMPLE_FACTOR 15

// Data format
const int DataWordSize = 16;
const int DataIntSize = 4;
const float DataMaxVal = pow(2.0, DataIntSize-1);
typedef ap_fixed<DataWordSize, DataIntSize, AP_RND, AP_SAT, 0> data_type;

// AXI4-Stream with sidechannels
typedef ap_axiu<DataWordSize, 1, 1, 1> axis_data;

// Function prototype
void downsampler(hls::stream<axis_data> &input_stream,
                 hls::stream<axis_data> &output_stream);

#endif // DOWNSAMPLER_HPP
