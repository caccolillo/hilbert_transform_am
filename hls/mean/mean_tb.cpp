#include <stdio.h>
#include <stdint.h>
#include "mean.hpp"


int main() {
    const int NUM_SAMPLES = 30;
    
    std::cout << "Mean Function Testbench\n";
    std::cout << "=======================\n\n";
    
    // Test 1: Constant input - Cumulative Mean
    std::cout << "Test 1: Constant Input (value = 10) - Cumulative Mean\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Mean Output"
              << std::setw(15) << "Expected\n";
    std::cout << std::string(55, '-') << "\n";
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        data_type input = 3.0;
        data_type output = mean(input);
        data_type expected = 3.0;
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(4) << input
                  << std::setw(15) << output
                  << std::setw(15) << expected
                  << "\n";
    }
    
    std::cout << "\n\n";
    
    // Test 2: Ramp input - Moving Average
    std::cout << "Test 2: Ramp Input (0, 1, 2, ...) - Moving Average\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Mean Output"
              << std::setw(20) << "Expected (approx)\n";
    std::cout << std::string(60, '-') << "\n";
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        data_type input = static_cast<data_type>(i);
        data_type output = mean(input);
        
        // Calculate expected value
        data_type expected;
        if (i < 15) {
            // During fill-up: mean of 0 to i
            expected = i / 2.0;
        } else {
            // Full window: mean of (i-14) to i
            expected = (i - 7.0);
        }
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(4) << input
                  << std::setw(15) << output
                  << std::setw(20) << expected
                  << "\n";
    }
    
    std::cout << "\n\n";
    
    // Test 3: Alternating values - Moving Average
    std::cout << "Test 3: Alternating Input (0, 10, 0, 10, ...) - Moving Average\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Mean Output\n";
    std::cout << std::string(40, '-') << "\n";
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        data_type input = (i % 2 == 0) ? 0.0 : 10.0;
        data_type output = mean(input);
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(4) << input
                  << std::setw(15) << output
                  << "\n";
    }
    
    std::cout << "\n";
    std::cout << "Expected: Output should stabilize around 5.0 after window fills\n\n";
    
    
    return 0;
}
