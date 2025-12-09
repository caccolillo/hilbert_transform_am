// ============================================================================
// downsampler.cpp
// ============================================================================
#include "downsampler.hpp"

void downsampler(hls::stream<axis_data> &input_stream,
                 hls::stream<axis_data> &output_stream) {
#pragma HLS INTERFACE axis port=input_stream
#pragma HLS INTERFACE axis port=output_stream
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    static int count = 0;
    static axis_data stored_sample;
    static bool pending_last = false;
    
    // Read input
    axis_data input_val;
    if (!input_stream.empty()) {
        input_stream.read(input_val);

        // Store first sample of each decimation period
        if (count == 0) {
            stored_sample = input_val;
        }

        // Track if we've seen TLAST in this decimation period
        if (input_val.last) {
            pending_last = true;
        }

        // Increment counter
        count++;

        // Output decimated sample when counter reaches DOWNSAMPLE_FACTOR
        if (count >= DOWNSAMPLE_FACTOR) {
            axis_data output_val;
            output_val.data = stored_sample.data;
            output_val.keep = stored_sample.keep;
            output_val.strb = stored_sample.strb;
            output_val.user = stored_sample.user;
            output_val.last = pending_last;  // Set TLAST if we saw it in this period
            output_val.id = stored_sample.id;
            output_val.dest = stored_sample.dest;

            output_stream.write(output_val);
            count = 0;
            pending_last = false;  // Clear the flag after outputting
        }
    }
}
