/*
 * AM Demodulator Testbench with AXI Stream Interface
 *
 * Description:
 * This testbench validates an AM (Amplitude Modulation) demodulator implemented
 * using the Hilbert transform approach with AXI Stream interfaces.
 * The demodulator processes AM signals through the following pipeline:
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
 * The signal is normalized to ensure -1 < am_signal < 1.
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
//
//#include "am_demodulator.hpp"
//#include <iostream>
//#include <fstream>
//#include <cmath>
//#include <vector>
//
//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
//
//#define INITIAL_DELAY 436
//
//// Test parameters
//const int NUM_SAMPLES = 1000;
//const float SAMPLE_RATE = 480000.0;  // 480 kHz sampling rate
//const float CARRIER_FREQ = 100000.0; // 100 kHz carrier
//const float MESSAGE_FREQ = 1000.0;   // 1 kHz message signal
//const float MODULATION_INDEX = 0.8;  // 80% modulation depth
//
//int main() {
//    std::cout << "========================================" << std::endl;
//    std::cout << "AM Demodulator Testbench (AXI Stream)" << std::endl;
//    std::cout << "========================================" << std::endl;
//    std::cout << "Sample Rate: " << SAMPLE_RATE << " Hz" << std::endl;
//    std::cout << "Carrier Frequency: " << CARRIER_FREQ << " Hz" << std::endl;
//    std::cout << "Message Frequency: " << MESSAGE_FREQ << " Hz" << std::endl;
//    std::cout << "Modulation Index: " << MODULATION_INDEX << std::endl;
//    std::cout << "Number of Samples: " << NUM_SAMPLES << std::endl;
//
//    // Calculate normalization factor to keep signal in range [-1, 1]
//    float max_envelope = 1.0 + MODULATION_INDEX;  // Maximum envelope value
//    float normalization_factor = 1.0 / max_envelope;
//    std::cout << "Normalization Factor: " << normalization_factor << std::endl;
//    std::cout << "Signal Range: [-" << max_envelope * normalization_factor
//              << ", " << max_envelope * normalization_factor << "]" << std::endl;
//    std::cout << "========================================" << std::endl;
//
//    // Create AXI Stream instances
//    hls::stream<axis_data> input_stream("input_stream");
//    hls::stream<axis_data> output_stream("output_stream");
//
//    // Buffer to store output samples
//    std::vector<float> output_buffer;
//    output_buffer.reserve(NUM_SAMPLES);
//
//    // Open output files for analysis
//    std::ofstream input_file("am_input.dat");
//    std::ofstream output_file("am_output.dat");
//    std::ofstream message_file("message_signal.dat");
//
//    if (!input_file.is_open() || !output_file.is_open() || !message_file.is_open()) {
//        std::cerr << "Error: Could not open output files!" << std::endl;
//        return -1;
//    }
//
//    // Test counters
//    int test_passed = 0;
//    int test_failed = 0;
//    float max_error = 0.0;
//    float sum_squared_error = 0.0;
//
//    std::cout << "\nGenerating AM signal and processing..." << std::endl;
//
//    // Generate and send all input samples to the stream
//    for (int i = 0; i < NUM_SAMPLES; i++) {
//        float t = (float)i / SAMPLE_RATE;
//
//        // Generate message signal (baseband)
//        float message = cos(2.0 * M_PI * MESSAGE_FREQ * t);
//
//        // Generate AM modulated signal: s(t) = (1 + m*message(t)) * cos(2*pi*fc*t)
//        // Normalize to keep signal in range [-1, 1]
//        float am_signal = (1.0 + MODULATION_INDEX * message) * cos(2.0 * M_PI * CARRIER_FREQ * t) * normalization_factor;
//
//        // Convert to fixed-point
//        data_type input_sample = am_signal;
//
//        // Create AXI Stream packet
//        axis_data input_packet;
//
//        // The .to_ap_int() method performs a numerical conversion (truncation), losing
//        // the fractional part. The correct way is to reinterpret the bits of the ap_fixed
//        // type as an ap_int to preserve the full precision across the AXI stream.
//        input_packet.data = *reinterpret_cast<ap_int<DataWordSize>*>(&input_sample);
//
//        input_packet.keep = -1;  // All bytes valid
//        input_packet.last = (i == NUM_SAMPLES - 1) ? 1 : 0;  // Mark last sample
//
//        // Write to input stream
//        input_stream.write(input_packet);
//
//
//        // Write input and message to files immediately
//        input_file << i << " " << am_signal << std::endl;
//        message_file << i << " " << message * MODULATION_INDEX * normalization_factor << std::endl;
//
//
//        // Call the demodulator (processes one sample per call)
//        am_demodulator(input_stream, output_stream);
//
//        // Progress indicator
//        if ((i + 1) % 1000 == 0) {
//            std::cout << "  Processed " << (i + 1) << " samples..." << std::endl;
//        }
//
//        if (!output_stream.empty()) {
//            axis_data output_packet;
//            output_stream.read(output_packet);
//
//            // To correctly convert from the AXI stream payload back to ap_fixed,
//            // we must reinterpret the raw ap_int bits as a data_type (ap_fixed),
//            // which is the reverse of the packing operation. A direct cast is a numerical
//            // conversion and will result in incorrect values.
//            ap_int<DataWordSize> temp_output_data = output_packet.data;
//            data_type output_sample = *reinterpret_cast<data_type*>(&temp_output_data);
//
//            #ifdef DEBUG
//              float output_float = output_sample;
//            #else
//              float output_float = output_sample.to_float();
//            #endif
//
//            // Store output in buffer
//            output_buffer.push_back(output_float);
//
//            // Error checking (skip initial transient - first 500 samples)
//            if (i > 500) {
//                float t = (float)i / SAMPLE_RATE;
//                float message = cos(2.0 * M_PI * MESSAGE_FREQ * t);
//                float expected = message * MODULATION_INDEX * normalization_factor;
//                float error = fabs(output_float - expected);
//
//                if (error > max_error) {
//                    max_error = error;
//                }
//                sum_squared_error += error * error;
//
//                // Check if output is within reasonable bounds
//                if (error < 0.3) {  // Tolerance for demodulation error
//                    test_passed++;
//                } else {
//                    test_failed++;
//                }
//            }
//        } else {
//            std::cerr << "Warning: Output stream empty at sample " << i << std::endl;
//            output_buffer.push_back(0.0);
//        }
//    }
//
//    // Write output with INITIAL_DELAY-sample delay (prepend INITIAL_DELAY zeros, skip last INITIAL_DELAY samples)
//    for (int i = 0; i < INITIAL_DELAY; i++) {
//        output_file << i << " " << 0.0 << std::endl;
//    }
//    for (int i = 0; i < NUM_SAMPLES - INITIAL_DELAY; i++) {
//        output_file << (i + INITIAL_DELAY) << " " << output_buffer[i] << std::endl;
//    }
//
//    // Close files
//    input_file.close();
//    output_file.close();
//    message_file.close();
//
//    // Calculate statistics
//    int total_checked = test_passed + test_failed;
//    float mse = (total_checked > 0) ? (sum_squared_error / total_checked) : 0.0;
//    float rmse = sqrt(mse);
//
//    // Print results
//    std::cout << "\n========================================" << std::endl;
//    std::cout << "Test Results:" << std::endl;
//    std::cout << "========================================" << std::endl;
//    std::cout << "Samples checked: " << total_checked << std::endl;
//    if (total_checked > 0) {
//        std::cout << "Passed: " << test_passed << " ("
//                  << (100.0 * test_passed / total_checked) << "%)" << std::endl;
//        std::cout << "Failed: " << test_failed << " ("
//                  << (100.0 * test_failed / total_checked) << "%)" << std::endl;
//    }
//    std::cout << "Maximum Error: " << max_error << std::endl;
//    std::cout << "RMSE: " << rmse << std::endl;
//    std::cout << "========================================" << std::endl;
//
//    // Determine overall pass/fail
//    bool overall_pass = (total_checked > 0) &&
//                        (test_failed < total_checked * 0.1) &&
//                        (max_error < 0.5);
//
//    if (overall_pass) {
//        std::cout << "\n*** TEST PASSED ***" << std::endl;
//        std::cout << "The AM demodulator is working correctly!" << std::endl;
//    } else {
//        std::cout << "\n*** TEST FAILED ***" << std::endl;
//        std::cout << "The demodulator output has excessive errors." << std::endl;
//    }
//
//    std::cout << "\nOutput files generated:" << std::endl;
//    std::cout << "  - am_input.dat (input AM signal)" << std::endl;
//    std::cout << "  - am_output.dat (demodulated output)" << std::endl;
//    std::cout << "  - message_signal.dat (expected output)" << std::endl;
//    std::cout << "\nYou can plot these with gnuplot or MATLAB/Python" << std::endl;
//    std::cout << "Example gnuplot command:" << std::endl;
//    std::cout << "  plot 'am_output.dat' w l, 'message_signal.dat' w l" << std::endl;
//
//    // Run Python plotting script
//    //system("python3 ./plot_csv.py");
//
//    return 0;
//}



/*
 * AM Demodulator Testbench (Cycle-stepped AXIS for HLS cosimulation)
 */

#include "am_demodulator.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define INITIAL_DELAY 436

// Test parameters
static const int NUM_SAMPLES = 1000;
static const float SAMPLE_RATE = 480000.0f;
static const float CARRIER_FREQ = 100000.0f;
static const float MESSAGE_FREQ = 1000.0f;
static const float MODULATION_INDEX = 0.8f;

int main() {

    std::cout << "========================================\n";
    std::cout << "AM Demodulator Testbench (AXI Stream)\n";
    std::cout << "========================================\n";
    std::cout << "Sample Rate: " << SAMPLE_RATE << " Hz\n";
    std::cout << "Carrier Frequency: " << CARRIER_FREQ << " Hz\n";
    std::cout << "Message Frequency: " << MESSAGE_FREQ << " Hz\n";
    std::cout << "Modulation Index: " << MODULATION_INDEX << "\n";
    std::cout << "Number of Samples: " << NUM_SAMPLES << "\n\n";

    float max_env = 1.0 + MODULATION_INDEX;
    float norm = 1.0 / max_env;

    // AXIS streams
    hls::stream<axis_data> input_stream("input_stream");
    hls::stream<axis_data> output_stream("output_stream");

    // File logging
    std::ofstream input_file("am_input.dat");
    std::ofstream output_file("am_output.dat");
    std::ofstream msg_file("message_signal.dat");

    if (!input_file || !output_file || !msg_file) {
        std::cerr << "ERROR opening output files!\n";
        return -1;
    }

    // Output buffer (for verification if needed)
    std::vector<float> output_buffer;
    output_buffer.reserve(NUM_SAMPLES);

    int samples_written = 0;
    int output_count = 0;

    // Run until:
    //   all samples have been pushed
    //   AND all output has been drained
    //
    // Provide generous cycle count because your demodulator has:
    //   - 16-tap delay
    //   - Hilbert 32-tap
    //   - envelope detector
    //   - IIR smoothing
    //   - downsampler (÷15)
    //
    const int MAX_CYCLES = NUM_SAMPLES * 50;  // safe upper bound

    for (int cycle = 0; cycle < MAX_CYCLES; cycle++) {

        // -------------------------------------------------------
        // Write input sample if stream is not full
        // -------------------------------------------------------
        if (samples_written < NUM_SAMPLES && !input_stream.full()) {
            int i = samples_written;
            float t = (float)i / SAMPLE_RATE;

            float message = cosf(2.0f * M_PI * MESSAGE_FREQ * t);
            float am_signal = (1.0f + MODULATION_INDEX * message)
                                * cosf(2.0f * M_PI * CARRIER_FREQ * t)
                                * norm;

            // Write logs
            input_file << i << " " << am_signal << "\n";
            msg_file   << i << " " << (message * MODULATION_INDEX * norm) << "\n";

            // Create AXIS word
            axis_data in_pkt;
            ap_int<DataWordSize> raw = *reinterpret_cast<ap_int<DataWordSize>*>(&am_signal);
            in_pkt.data = raw;
            in_pkt.keep = -1;
            in_pkt.last = (i == NUM_SAMPLES - 1);

            input_stream.write(in_pkt);
            samples_written++;
        }

        // -------------------------------------------------------
        // Advance hardware by one cycle
        // -------------------------------------------------------
        am_demodulator(input_stream, output_stream);

        // -------------------------------------------------------
        // Read output if available
        // -------------------------------------------------------
        while (!output_stream.empty()) {
            axis_data out_pkt;
            output_stream.read(out_pkt);

            ap_int<DataWordSize> raw = out_pkt.data;
            float out_value = *reinterpret_cast<float*>(&raw);

            output_file << output_count << " " << out_value << "\n";
            output_buffer.push_back(out_value);

            output_count++;
        }

        // Stop early if completely done:
        if (samples_written >= NUM_SAMPLES && output_stream.empty()) {
            // Give a few extra cycles for pipeline flush
            if (cycle > NUM_SAMPLES * 20)
                break;
        }
    }

    std::cout << "Cosimulation testbench finished.\n";
    std::cout << "Samples written: " << samples_written << "\n";
    std::cout << "Outputs produced: " << output_count << "\n";

    input_file.close();
    output_file.close();
    msg_file.close();

    return 0;
}
