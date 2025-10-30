#include <stdio.h>
#include <stdint.h>
#include "downsampler.hpp"

int main() {
    const int NUM_SAMPLES = 60; // 4 full cycles of downsampling
    const double FREQ = 1.0;     // 1 Hz test signal
    const double SAMPLE_RATE = 100.0; // 100 Hz sample rate
    
    std::cout << "Downsampler by 15 - Testbench\n";
    std::cout << "================================\n\n";
    
    // Test 1: Ramp input
    std::cout << "Test 1: Ramp Input\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Output" 
              << std::setw(10) << "Updated?\n";
    std::cout << std::string(50, '-') << "\n";
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        data_type input = static_cast<data_type>(i);
        data_type output = downsampler(input);
        bool updated = (i % 15 == 0);
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(2) << input
                  << std::setw(15) << output
                  << std::setw(10) << (updated ? "YES" : "no")
                  << "\n";
    }
    
    std::cout << "\n\n";
    
    // Test 2: Sine wave input
    std::cout << "Test 2: Sine Wave Input (1 Hz @ 100 Hz sample rate)\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Output" 
              << std::setw(10) << "Updated?\n";
    std::cout << std::string(50, '-') << "\n";
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        double t = i / SAMPLE_RATE;
        data_type input = static_cast<data_type>(sin(2 * M_PI * FREQ * t));
        data_type output = downsampler(input);
        bool updated = (i % 15 == 0);
        
        std::cout << std::setw(10) << i 
                  << std::setw(15) << std::fixed << std::setprecision(4) << input
                  << std::setw(15) << output
                  << std::setw(10) << (updated ? "YES" : "no")
                  << "\n";
    }
    
    std::cout << "\n";
    std::cout << "Expected behavior:\n";
    std::cout << "- Output should update every 15 samples (at samples 0, 15, 30, 45...)\n";
    std::cout << "- Output should hold constant between updates\n";
    std::cout << "- Effective output rate: " << SAMPLE_RATE / 15.0 << " Hz\n";
    
    return 0;
}