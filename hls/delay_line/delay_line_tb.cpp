#include <stdio.h>
#include <stdint.h>
#include "delay_line.hpp"


int main() {
    const int NUM_SAMPLES = 50;
    
    std::cout << "Delay Line (16 samples) Testbench\n";
    std::cout << "==================================\n\n";
    
    // Test 1: Impulse response
    std::cout << "Test 1: Impulse Response (single spike at sample 0)\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Output"
              << std::setw(20) << "Expected Output\n";
    std::cout << std::string(60, '-') << "\n";
    
    for (int i = 0; i < 30; i++) {
        data_type input = (i == 0) ? 1.0 : 0.0;
        data_type output = delay_line(input);
        data_type expected = (i == 16) ? 1.0 : 0.0;
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(1) << input
                  << std::setw(15) << output
                  << std::setw(20) << expected;
        
        if (i == 16) {
            std::cout << " <- Impulse appears here!";
        }
        std::cout << "\n";
    }
    
    std::cout << "\n\n";
    
    
    // Test 2: Sine wave
    std::cout << "Test 4: Sine Wave Input (shows phase shift)\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Output"
              << std::setw(15) << "Difference\n";
    std::cout << std::string(55, '-') << "\n";
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        data_type input = sin(2 * M_PI * i / 32.0);
        data_type output = delay_line(input);
        data_type diff = input - output;
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(4) << input
                  << std::setw(15) << output
                  << std::setw(15) << diff
                  << "\n";
    }
    
    std::cout << "\n";
    std::cout << "Summary:\n";
    std::cout << "- Delay length: 16 samples\n";
    std::cout << "- First 16 outputs are zeros (initial buffer state)\n";
    std::cout << "- After sample 16, output = input delayed by 16 samples\n";
    std::cout << "- For sine wave: 16 samples delay = 180° phase shift at 32-sample period\n";
    
    return 0;
}
