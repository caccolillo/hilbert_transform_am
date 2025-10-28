#include "envelope_detector.hpp"

// Top-level function suitable for Vitis HLS
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
