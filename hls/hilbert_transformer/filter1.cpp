#include "filter1.hpp"



// Section 2: static internal state
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
	for (int i = 0; i < FILTER_LENGTH; i++)
		acc += delay_pipeline[i] * coeffs[i];

    return acc;

}
