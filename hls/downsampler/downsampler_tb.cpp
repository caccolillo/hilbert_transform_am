// downsampler_tb.cpp
#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include "downsampler.hpp"

int main() {
    const int NUM_SAMPLES = 60; // 4 full cycles of downsampling
    const double FREQ = 1.0;     // 1 Hz test signal
    const double SAMPLE_RATE = 100.0; // 100 Hz sample rate
    
    std::cout << "Downsampler by 15 - Testbench\n";
    std::cout << "================================\n\n";
    
    // Create streams
    hls::stream<axis_data> input_stream("input");
    hls::stream<axis_data> output_stream("output");

    // Test 1: Ramp input
    std::cout << "Test 1: Ramp Input\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Output" 
              << std::setw(10) << "TLAST"
              << std::setw(10) << "Valid\n";
    std::cout << std::string(60, '-') << "\n";
    
    // Push ramp inputs
    for (int i = 0; i < NUM_SAMPLES; i++) {
        axis_data input_val;
        input_val.data = static_cast<data_type>(i);
        input_val.last = (i == NUM_SAMPLES - 1) ? 1 : 0;  // TLAST on final sample
        input_val.keep = 0x3;
        input_val.strb = 0x3;
        input_val.user = 0;
        input_val.id = 0;
        input_val.dest = 0;
        input_stream.write(input_val);
    }

    // Process and read outputs
    data_type last_output = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        downsampler(input_stream, output_stream);

        bool has_output = !output_stream.empty();
        axis_data output_val;
        
        if (has_output) {
            output_stream.read(output_val);
            last_output = output_val.data;

            std::cout << std::setw(10) << i
                      << std::setw(15) << std::fixed << std::setprecision(2) << static_cast<data_type>(i)
                      << std::setw(15) << output_val.data
                      << std::setw(10) << (output_val.last ? "YES" : "no")
                      << std::setw(10) << "YES\n";
        } else {
            std::cout << std::setw(10) << i
                      << std::setw(15) << std::fixed << std::setprecision(2) << static_cast<data_type>(i)
                      << std::setw(15) << last_output
                      << std::setw(10) << "-"
                      << std::setw(10) << "no\n";
        }
    }
    
    std::cout << "\n\n";
    
    // Test 2: Sine wave input with packet boundaries
    std::cout << "Test 2: Sine Wave Input with Packet Boundaries\n";
    std::cout << "(Packet 1: 0-29, Packet 2: 30-59)\n";
    std::cout << std::setw(10) << "Sample#" 
              << std::setw(15) << "Input" 
              << std::setw(15) << "Output" 
              << std::setw(10) << "TLAST"
              << std::setw(10) << "Valid\n";
    std::cout << std::string(60, '-') << "\n";
    
    // Push sine wave inputs with TLAST at sample 29 and 59
    for (int i = 0; i < NUM_SAMPLES; i++) {
        axis_data input_val;
        double t = i / SAMPLE_RATE;
        input_val.data = static_cast<data_type>(sin(2 * M_PI * FREQ * t));
        input_val.last = (i == 29 || i == NUM_SAMPLES - 1) ? 1 : 0;
        input_val.keep = 0x3;
        input_val.strb = 0x3;
        input_val.user = 0;
        input_val.id = 0;
        input_val.dest = 0;
        input_stream.write(input_val);
    }

    // Process and read outputs
    last_output = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        downsampler(input_stream, output_stream);

        bool has_output = !output_stream.empty();
        axis_data output_val;

        double t = i / SAMPLE_RATE;
        data_type input = static_cast<data_type>(sin(2 * M_PI * FREQ * t));
        
        if (has_output) {
            output_stream.read(output_val);
            last_output = output_val.data;

            std::cout << std::setw(10) << i
                      << std::setw(15) << std::fixed << std::setprecision(4) << input
                      << std::setw(15) << output_val.data
                      << std::setw(10) << (output_val.last ? "YES" : "no")
                      << std::setw(10) << "YES\n";
        } else {
            std::cout << std::setw(10) << i
                      << std::setw(15) << std::fixed << std::setprecision(4) << input
                      << std::setw(15) << last_output
                      << std::setw(10) << "-"
                      << std::setw(10) << "no\n";
        }
    }
    
    std::cout << "\n";
    std::cout << "Expected behavior:\n";
    std::cout << "- Output should be valid every 15 samples (at samples 14, 29, 44, 59...)\n";
    std::cout << "- TLAST should appear in output at samples 29 and 59 (packet boundaries)\n";
    std::cout << "- No gaps in decimation across packet boundaries\n";
    std::cout << "- Effective output rate: " << SAMPLE_RATE / 15.0 << " Hz\n";
    
    return 0;
}
