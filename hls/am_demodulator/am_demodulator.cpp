#include "am_demodulator.hpp"



data_type delay_line(data_type filter_in) {
    const int DELAY_SAMPLES = 16;
    static data_type buffer[DELAY_SAMPLES] = {0};
    static int index = 0;
    
    // Get the delayed output (oldest sample in buffer)
    data_type output = buffer[index];
    
    // Store new input sample
    buffer[index] = filter_in;
    
    // Update circular buffer index
    index = (index + 1) % DELAY_SAMPLES;
    
    return output;
}

data_type downsampler(data_type filter_in) {
    static int count = 0;
    static data_type sample = 0;
    
    if (count == 0) {
        sample = filter_in;
    }
    
    count++;
    if (count >= 15) {
        count = 0;
    }
    
    return sample;
}

data_type envelope_detector(data_type x, data_type h) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II=1
    data_type x_sq = x * x;
    data_type h_sq = h * h;
    data_type sum = x_sq + h_sq;

    // Avoid invalid negative or zero input
    if (sum <= 0)
        return 0;

    // Initial approximation - use actual division
    data_type y = sum / 2;  // NOT sum >> 1
    if (y == 0) y = 1;

    // Newton-Raphson: y_new = 0.5 * (y + sum/y)
    LOOP_SQRT:
    for (int i = 0; i < 10; i++) {
#pragma HLS PIPELINE
        data_type quotient = sum / y;
        data_type sum_temp = y + quotient;
        y = sum_temp / 2;  // Use division, NOT >> 1
    }
    return y;
}


data_type applyIIRFilter(data_type filter_in) {
	static bool initialized = false;
	static data_type delay0;
	static data_type delay1;
	static data_type output_register;

	// One-time initialization
	if (!initialized) {
		delay0 = 0.0;
		delay1 = 0.0;
		output_register = 0.0;
		initialized = true;
	}

	// Signal path logic
	data_type input_register = filter_in;
	accum_type scale1 = input_register * scaleconst1;
	accum_type inputconv1 = scale1;

	accum_type a2mul1 = delay0 * coeff_a2_section1;
	accum_type a3mul1 = delay1 * coeff_a3_section1;
	accum_type b2mul1 = delay0 * coeff_b2_section1;

	accum_type a2sum1 = inputconv1 - a2mul1;
	accum_type a1sum1 = a2sum1 - a3mul1;
	accum_type b2sum1 = a1sum1 + b2mul1;
	accum_type b1sum1 = b2sum1 + delay1;

	output_register = b1sum1;

	// Update delay line
	delay1 = delay0;
	delay0 = a1sum1;

	//return output sample
	return output_register;

}

data_type filter1(data_type input) {
	static bool initialized = false;
	static data_type delay_pipeline[FILTER_LENGTH];
	static data_type d2;
	static accum_type acc = 0.0;


	// One-time initialization
	if (!initialized) {
		acc = 0.0;
		for(int i = 0; i < FILTER_LENGTH; i ++ )
		  delay_pipeline[i] = 0.0;
		initialized = true;
	}


	// Shift the delay line
	for (int i = FILTER_LENGTH - 1; i > 0; i--)
		delay_pipeline[i] = delay_pipeline[i - 1];
	delay_pipeline[0] = input;

	// Compute convolution (dot product)
	acc = 0.0; //reset accumulator
	for (int i = 0; i < FILTER_LENGTH; i++)
		acc += delay_pipeline[i] * coeffs[i];

    return acc;

}

//data_type mean(data_type x) {
//    static data_type y = 0;
//    const data_type alpha = 0.0001; // smaller alpha → slower, smoother DC estimate
//#pragma HLS INLINE off
//    y = y + alpha * (x - y);
//    return y;
//}

//data_type mean(data_type x) {
//    static data_type y = 0;
//    const int ALPHA_SHIFT = 12;
//#pragma HLS INLINE off
//#pragma HLS RESET variable=y
//#pragma HLS PIPELINE II=1
//
//    data_type error = x - y;
//    data_type update = error >> ALPHA_SHIFT;
//    y = y + update;
//    return y;
//}

//
//data_type mean(data_type x) {
//    static data_type y = 0;
//    const data_type alpha = 0.0001;
//#pragma HLS INLINE off
//#pragma HLS RESET variable=y
//#pragma HLS PIPELINE II=1
//
//    data_type error = x - y;
//    data_type update = error * alpha;
//    y = y + update;
//    return y;
//}


//data_type mean(data_type x) {
//    static data_type y = 0;
//    const data_type alpha = 0.0001;
//    data_type error;
//    data_type update;
//#pragma HLS INLINE off
//#pragma HLS RESET variable=y
//#pragma HLS PIPELINE II=1
//
//
//
//#pragma HLS BIND_OP variable=error op=sub impl=dsp
//#pragma HLS BIND_OP variable=update op=mul impl=dsp
//#pragma HLS BIND_OP variable=y op=add impl=dsp
//
//
//    error = x - y;
//    update = error * alpha;
//    y = y + update;
//
//    return y;
//}


data_type mean(data_type x) {
    static data_type y = 0;
    const data_type alpha = 0.0001;

#pragma HLS INLINE off
#pragma HLS RESET variable=y
#pragma HLS PIPELINE II=1
    data_type error = x - y;
    data_type update = error * alpha;
    // Modern Vitis HLS approach
#pragma HLS RESOURCE variable=update core=DSP48
#pragma HLS RESOURCE variable=error core=DSP48


    y = y + update;

    return y;
}

void am_demodulator(hls::stream<axis_data> &input_stream,
                    hls::stream<axis_data> &output_stream) {
#pragma HLS INTERFACE axis port=input_stream
#pragma HLS INTERFACE axis port=output_stream
#pragma HLS INTERFACE ap_ctrl_none port=return
#pragma HLS PIPELINE II=1

    axis_data input_packet;
    axis_data output_packet;

    // Read from input stream (blocking read)
    input_stream.read(input_packet);

    // =============================================================================
    // Unpack the input data from the stream using reinterpret_cast.
    // A direct assignment `data_type filter_in = input_packet.data;` would be a
    // numerical conversion, corrupting the fixed-point value.
    // =============================================================================
    ap_int<DataWordSize> temp_input_data = input_packet.data;
    data_type filter_in = *reinterpret_cast<data_type*>(&temp_input_data);


    // Processing pipeline
    data_type delayed_signal = 0.0;
    data_type hilbert_signal = 0.0;
    data_type envelope = 0.0;
    data_type filtered_envelope = 0.0;
    data_type downsampled_output = 0.0;
    data_type mean_val = 0.0;
    data_type final_output = 0.0;
    const data_type gain1 = 1.0E+00;
    const data_type gain2 = 1.0E+00;
    const data_type gain3 = 1.0E+00;

    filter_in = filter_in * gain1 * gain2;

    // Step 1: Delay input by 16 samples
    delayed_signal = delay_line(filter_in);

    // Step 2: Compute Hilbert transform
    hilbert_signal = filter1(filter_in);

    // Step 3: Envelope detection (magnitude)
    envelope = envelope_detector(delayed_signal, hilbert_signal);
    envelope *= gain1;

    // Step 4: Smooth envelope with IIR filter
    filtered_envelope = applyIIRFilter(envelope);
    filtered_envelope *= gain2;

    // Step 5: Downsample the smoothed envelope
    downsampled_output = downsampler(filtered_envelope);

    // Step 6: Compute mean
    mean_val = mean(downsampled_output);
    mean_val = mean_val * gain3;

    // Step 7: Subtract mean from downsampled output
    final_output = downsampled_output - mean_val;


    // =============================================================================
    // FIXED: Pack the output data into the stream using reinterpret_cast.
    // A direct assignment `output_packet.data = final_output;` would truncate
    // the fixed-point value to an integer, losing all fractional precision.
    // =============================================================================
    output_packet.data = *reinterpret_cast<ap_int<DataWordSize>*>(&final_output);

    output_packet.keep = -1; // All bytes valid
    output_packet.last = input_packet.last;

    // Write to output stream
    output_stream.write(output_packet);
}
