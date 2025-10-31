/*
 * AM Demodulator Testbench
 * 
 * Description:
 * This testbench validates an AM (Amplitude Modulation) demodulator implemented
 * using the Hilbert transform approach. The demodulator processes AM signals
 * through the following pipeline:
 * 
 * 1. Delay Line: 16-sample delay to compensate for Hilbert transform group delay
 * 2. Hilbert Transform: 32-tap FIR filter to generate quadrature component
 * 3. Envelope Detection: Computes magnitude using sqrt(I^2 + Q^2) with 
 *    Newton-Raphson approximation for square root
 * 4. IIR Smoothing Filter: Lowpass filter to smooth the envelope
 * 5. Downsampler: Reduces sample rate by factor of 15
 * 6. DC Removal: Subtracts running mean to remove DC offset
 * 
 * The testbench generates a standard AM signal: s(t) = (1 + m*cos(2πf_m*t)) * cos(2πf_c*t)
 * where m is the modulation index, f_m is the message frequency, and f_c is the carrier.
 * 
 * Test Parameters:
 * - Sampling Rate: 480 kHz
 * - Carrier Frequency: 100 kHz
 * - Message Frequency: 1 kHz (audio tone)
 * - Modulation Index: 0.8 (80%)
 * 
 * Output:
 * - Console output with pass/fail statistics and error metrics
 * - Data files for visualization and further analysis
 */

#include "am_demodulator.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Test parameters
const int NUM_SAMPLES = 4000;
const float SAMPLE_RATE = 480000.0;  // 480 kHz sampling rate
const float CARRIER_FREQ = 100000.0;  // 100 kHz carrier
const float MESSAGE_FREQ = 1000.0;    // 1 kHz message signal
const float MODULATION_INDEX = 0.8;   // 80% modulation depth

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "AM Demodulator Testbench" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Sample Rate: " << SAMPLE_RATE << " Hz" << std::endl;
    std::cout << "Carrier Frequency: " << CARRIER_FREQ << " Hz" << std::endl;
    std::cout << "Message Frequency: " << MESSAGE_FREQ << " Hz" << std::endl;
    std::cout << "Modulation Index: " << MODULATION_INDEX << std::endl;
    std::cout << "Number of Samples: " << NUM_SAMPLES << std::endl;
    std::cout << "========================================" << std::endl;

    // Open output files for analysis
    std::ofstream input_file("am_input.dat");
    std::ofstream output_file("am_output.dat");
    std::ofstream message_file("message_signal.dat");
    
    if (!input_file.is_open() || !output_file.is_open() || !message_file.is_open()) {
        std::cerr << "Error: Could not open output files!" << std::endl;
        return -1;
    }

    // Test counters
    int test_passed = 0;
    int test_failed = 0;
    float max_error = 0.0;
    float sum_squared_error = 0.0;
    
    std::cout << "\nGenerating AM signal and processing..." << std::endl;
    
    // Process samples through the demodulator
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float t = (float)i / SAMPLE_RATE;
        
        // Generate message signal (baseband)
        float message = cos(2.0 * M_PI * MESSAGE_FREQ * t);
        
        // Generate AM modulated signal: s(t) = (1 + m*message(t)) * cos(2*pi*fc*t)
        float am_signal = (1.0 + MODULATION_INDEX * message) * cos(2.0 * M_PI * CARRIER_FREQ * t);
        
        // Convert to fixed-point
        data_type input_sample = am_signal;
        
        // Run through demodulator
        data_type output_sample = am_demodulator(input_sample);
        
        // Convert back to float for analysis
        float output_float = output_sample.to_float();
        
        // Write to files
        input_file << i << " " << am_signal << std::endl;
        output_file << i << " " << output_float << std::endl;
        message_file << i << " " << message * MODULATION_INDEX << std::endl;
        
        // Error checking (skip initial transient - first 500 samples)
        if (i > 500) {
            float expected = message * MODULATION_INDEX;
            float error = fabs(output_float - expected);
            
            if (error > max_error) {
                max_error = error;
            }
            sum_squared_error += error * error;
            
            // Check if output is within reasonable bounds
            if (error < 0.3) {  // Tolerance for demodulation error
                test_passed++;
            } else {
                test_failed++;
            }
        }
        
        // Progress indicator
        if ((i + 1) % 1000 == 0) {
            std::cout << "  Processed " << (i + 1) << " samples..." << std::endl;
        }
    }
    
    // Close files
    input_file.close();
    output_file.close();
    message_file.close();
    
    // Calculate statistics
    int total_checked = test_passed + test_failed;
    float mse = sum_squared_error / total_checked;
    float rmse = sqrt(mse);
    
    // Print results
    std::cout << "\n========================================" << std::endl;
    std::cout << "Test Results:" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Samples checked: " << total_checked << std::endl;
    std::cout << "Passed: " << test_passed << " (" 
              << (100.0 * test_passed / total_checked) << "%)" << std::endl;
    std::cout << "Failed: " << test_failed << " (" 
              << (100.0 * test_failed / total_checked) << "%)" << std::endl;
    std::cout << "Maximum Error: " << max_error << std::endl;
    std::cout << "RMSE: " << rmse << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Determine overall pass/fail
    bool overall_pass = (test_failed < total_checked * 0.1) && (max_error < 0.5);
    
    if (overall_pass) {
        std::cout << "\n*** TEST PASSED ***" << std::endl;
        std::cout << "The AM demodulator is working correctly!" << std::endl;
    } else {
        std::cout << "\n*** TEST FAILED ***" << std::endl;
        std::cout << "The demodulator output has excessive errors." << std::endl;
    }
    
    std::cout << "\nOutput files generated:" << std::endl;
    std::cout << "  - am_input.dat (input AM signal)" << std::endl;
    std::cout << "  - am_output.dat (demodulated output)" << std::endl;
    std::cout << "  - message_signal.dat (expected output)" << std::endl;
    std::cout << "\nYou can plot these with gnuplot or MATLAB/Python" << std::endl;
    std::cout << "Example gnuplot command:" << std::endl;
    std::cout << "  plot 'am_output.dat' w l, 'message_signal.dat' w l" << std::endl;
    
    // Run Python plotting script
    system("python3 ./plot_csv.py");

    return overall_pass ? 0 : 1;
}
