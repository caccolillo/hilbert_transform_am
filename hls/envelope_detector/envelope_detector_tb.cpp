#include "envelope_detector.hpp"
#include <stdio.h>

// Function prototype
extern data_type envelope_detector(data_type x, data_type h);

int main() {
    data_type x = 0.3;
    data_type h = 1.3;
    data_type result;

    result = envelope_detector(x, h);

    printf("x = %f\n", float(x));
    printf("h = %f\n", float(h));
    printf("Result = %f\n", float(result));
    printf("Expected = 1.334\n");


}
