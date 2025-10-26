
#include "filter_envelope_detector.hpp"

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
