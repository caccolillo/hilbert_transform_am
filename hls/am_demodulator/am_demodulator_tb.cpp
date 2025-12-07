/*
 * AM Demodulator Testbench with AXI Stream Interface - FIXED VERSION
 *
 * Key fixes:
 * 1. Accounts for correct pipeline latency in INITIAL_DELAY
 * 2. Properly handles the new output validity logic
 * 3. Correctly aligns outputs to expected values
 * 4. Handles backpressure scenarios properly
 */

#include "am_demodulator.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// FIXED: Must match PIPELINE_LATENCY in am_demodulator.cpp
// PIPELINE_LATENCY = 16 + FILTER_LENGTH + 4
// Assuming FILTER_LENGTH = 32 (typical Hilbert filter), total = 52
// Add downsampling offset if applicable
#define PIPELINE_LATENCY_DELAY (16 + FILTER_LENGTH + 4)
#define INITIAL_DELAY PIPELINE_LATENCY_DELAY

// Test parameters
const int NUM_SAMPLES = 600; // Use 60000 for full test, 600 for quick test
const float SAMPLE_RATE = 480000.0f;
const float CARRIER_FREQ = 100000.0f;
const float MESSAGE_FREQ = 1000.0f;
const float MODULATION_INDEX = 0.8f;

// Helper: bit-preserving pack/unpack
static inline ap_int<DataWordSize> pack_to_apint(const data_type &v) {
    ap_int<DataWordSize> out;
    std::memset(&out, 0, sizeof(out));
    size_t n = std::min(sizeof(out), sizeof(v));
    std::memcpy(&out, &v, n);
    return out;
}

static inline data_type unpack_from_apint(const ap_int<DataWordSize> &v) {
    data_type out;
    std::memset(&out, 0, sizeof(out));
    size_t n = std::min(sizeof(out), sizeof(v));
    std::memcpy(&out, &v, n);
    return out;
}

int main() {
    std::cout << "=== AM Demodulator Testbench (Fixed Version) ===" << std::endl;
    std::cout << "Samples: " << NUM_SAMPLES << ", Fs: " << SAMPLE_RATE << " Hz" << std::endl;
    std::cout << "Pipeline latency: " << PIPELINE_LATENCY_DELAY << " samples" << std::endl;

    float max_env = 1.0f + MODULATION_INDEX;
    float norm = 1.0f / max_env;

    hls::stream<axis_data> input_stream("input_stream");
    hls::stream<axis_data> output_stream("output_stream");

    std::vector<float> input_buffer;
    std::vector<float> output_buffer;
    std::vector<float> expected_buffer;

    input_buffer.reserve(NUM_SAMPLES);
    output_buffer.reserve(NUM_SAMPLES + 100); // Account for all outputs including pipeline fill
    expected_buffer.reserve(NUM_SAMPLES);

    // Generate all input samples and expected outputs
    std::cout << "Generating test signals..." << std::endl;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float t = (float)i / SAMPLE_RATE;
        float message = cosf(2.0f * M_PI * MESSAGE_FREQ * t);
        float am_signal = (1.0f + MODULATION_INDEX * message) * cosf(2.0f * M_PI * CARRIER_FREQ * t) * norm;

        input_buffer.push_back(am_signal);
        // Expected output is the demodulated message
        expected_buffer.push_back(message * MODULATION_INDEX * norm);
    }

    // Feed all inputs into the stream
    std::cout << "Writing " << NUM_SAMPLES << " samples to input stream..." << std::endl;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        data_type in_fixed = static_cast<data_type>(input_buffer[i]);
        axis_data pkt;
        pkt.data = pack_to_apint(in_fixed);
        pkt.keep = -1;
        pkt.last = (i == NUM_SAMPLES - 1) ? 1 : 0;
        input_stream.write(pkt);
    }

    // Process until all inputs consumed and outputs generated
    std::cout << "Processing through demodulator..." << std::endl;
    int iterations = 0;
    int last_input_size = NUM_SAMPLES;
    int last_output_size = 0;
    bool tlast_seen = false;
    int stall_count = 0;

    // Keep calling DUT until:
    // 1. Input stream is empty (all samples consumed)
    // 2. tlast has been seen in output (indicating last sample processed)
    while (!input_stream.empty() || !tlast_seen) {
        am_demodulator(input_stream, output_stream);
        iterations++;

        // Drain all available outputs
        while (!output_stream.empty()) {
            axis_data outpkt = output_stream.read();
            ap_int<DataWordSize> raw = outpkt.data;
            data_type out_ap = unpack_from_apint(raw);

            float out_float;
            #ifdef HLS_AP_FIXED
                out_float = out_ap.to_float();
            #else
                out_float = static_cast<float>(out_ap);
            #endif

            output_buffer.push_back(out_float);

            if (outpkt.last) {
                tlast_seen = true;
                std::cout << "tlast detected at output sample " << output_buffer.size() << std::endl;
            }
        }

        // Progress reporting every 10000 iterations
        if (iterations % 10000 == 0) {
            int current_input_size = input_stream.size();
            int current_output_size = output_buffer.size();
            std::cout << "Iter: " << iterations
                      << ", Input remaining: " << current_input_size
                      << ", Outputs: " << current_output_size << std::endl;

            // Deadlock detection - if no progress for multiple checks
            if (current_input_size == last_input_size &&
                current_output_size == last_output_size) {
                stall_count++;
                if (stall_count > 3) {
                    std::cerr << "ERROR: No progress detected for 30000 iterations - possible deadlock!" << std::endl;
                    std::cerr << "Input remaining: " << current_input_size << std::endl;
                    std::cerr << "Outputs produced: " << current_output_size << std::endl;
                    break;
                }
            } else {
                stall_count = 0; // Reset if progress was made
            }
            last_input_size = current_input_size;
            last_output_size = current_output_size;
        }

        // Safety limit - should never need this many iterations
        if (iterations > NUM_SAMPLES * 5) {
            std::cerr << "ERROR: Exceeded maximum iterations (" << NUM_SAMPLES * 5 << ") - breaking" << std::endl;
            break;
        }
    }

    std::cout << "\nProcessing complete!" << std::endl;
    std::cout << "Total iterations: " << iterations << std::endl;
    std::cout << "Outputs produced: " << output_buffer.size() << std::endl;
    std::cout << "Expected outputs: ~" << (NUM_SAMPLES - PIPELINE_LATENCY_DELAY) << " (after pipeline latency)" << std::endl;

    // Write output files for visualization
    std::cout << "\nWriting output files..." << std::endl;
    std::ofstream input_file("am_input.dat");
    std::ofstream output_file("am_output.dat");
    std::ofstream message_file("message_signal.dat");

    if (!input_file || !output_file || !message_file) {
        std::cerr << "Error opening output files!" << std::endl;
        return -1;
    }

    // Write input signal and expected message (reference)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        input_file << i << " " << input_buffer[i] << "\n";
        message_file << i << " " << expected_buffer[i] << "\n";
    }

    // Write outputs aligned to input indices
    // Outputs start appearing at index PIPELINE_LATENCY_DELAY
    for (int k = 0; k < (int)output_buffer.size(); k++) {
        int in_idx = k + PIPELINE_LATENCY_DELAY;
        if (in_idx < NUM_SAMPLES) {
            output_file << in_idx << " " << output_buffer[k] << "\n";
        }
    }

    input_file.close();
    output_file.close();
    message_file.close();

    // Calculate error metrics
    std::cout << "\nCalculating error metrics..." << std::endl;
    int test_passed = 0, test_failed = 0;
    float max_error = 0.0f;
    double sum_sq_err = 0.0;
    int samples_compared = 0;

    // Compare outputs to expected values
    // output[k] corresponds to input[k + PIPELINE_LATENCY_DELAY]
    for (int k = 0; k < (int)output_buffer.size(); k++) {
        int corresponding_input_idx = k + PIPELINE_LATENCY_DELAY;

        // Only compare if we have a valid expected value
        if (corresponding_input_idx >= 0 && corresponding_input_idx < NUM_SAMPLES) {
            float output_val = output_buffer[k];
            float expected = expected_buffer[corresponding_input_idx];
            float err = fabsf(output_val - expected);

            if (err > max_error) {
                max_error = err;
            }
            sum_sq_err += double(err) * double(err);
            samples_compared++;

            // Pass/fail threshold
            const float ERROR_THRESHOLD = 0.3f;
            if (err < ERROR_THRESHOLD) {
                test_passed++;
            } else {
                test_failed++;
                // Print first few failures for debugging
                if (test_failed <= 10) {
                    std::cout << "  Sample " << k << " (input idx " << corresponding_input_idx << "): "
                              << "output=" << output_val << ", expected=" << expected
                              << ", error=" << err << std::endl;
                }
            }
        }
    }

    // Print comprehensive results
    int total_checked = test_passed + test_failed;
    float mse = (samples_compared > 0) ? float(sum_sq_err / samples_compared) : 0.0f;
    float rmse = sqrtf(mse);

    std::cout << "\n========================================" << std::endl;
    std::cout << "=== TESTBENCH RESULTS ===" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Input samples: " << NUM_SAMPLES << std::endl;
    std::cout << "Output samples: " << output_buffer.size() << std::endl;
    std::cout << "Pipeline latency: " << PIPELINE_LATENCY_DELAY << " samples" << std::endl;
    std::cout << "Samples compared: " << samples_compared << std::endl;

    if (total_checked > 0) {
        float pass_rate = 100.0f * test_passed / total_checked;
        float fail_rate = 100.0f * test_failed / total_checked;

        std::cout << "\nAccuracy:" << std::endl;
        std::cout << "  Passed: " << test_passed << " (" << pass_rate << "%)" << std::endl;
        std::cout << "  Failed: " << test_failed << " (" << fail_rate << "%)" << std::endl;
    }

    std::cout << "\nError Metrics:" << std::endl;
    std::cout << "  Max error: " << max_error << std::endl;
    std::cout << "  RMSE: " << rmse << std::endl;
    std::cout << "  MSE: " << mse << std::endl;

    // Overall pass/fail criteria
    const float MAX_ALLOWED_ERROR = 0.5f;
    const float MAX_FAIL_RATE = 0.1f; // 10%

    bool overall_pass = (total_checked > 0) &&
                       (test_failed < total_checked * MAX_FAIL_RATE) &&
                       (max_error < MAX_ALLOWED_ERROR);

    std::cout << "\n========================================" << std::endl;
    if (overall_pass) {
        std::cout << "*** TESTBENCH PASSED ***" << std::endl;
    } else {
        std::cout << "*** TESTBENCH FAILED ***" << std::endl;
        if (test_failed >= total_checked * MAX_FAIL_RATE) {
            std::cout << "  Reason: Fail rate too high (" << (100.0f * test_failed / total_checked) << "% > 10%)" << std::endl;
        }
        if (max_error >= MAX_ALLOWED_ERROR) {
            std::cout << "  Reason: Max error too large (" << max_error << " >= " << MAX_ALLOWED_ERROR << ")" << std::endl;
        }
    }
    std::cout << "========================================" << std::endl;

    std::cout << "\nOutput files created:" << std::endl;
    std::cout << "  - am_input.dat (input AM signal)" << std::endl;
    std::cout << "  - am_output.dat (demodulated output)" << std::endl;
    std::cout << "  - message_signal.dat (expected reference)" << std::endl;

    // Optionally run plotting script
    std::cout << "\nTo visualize results, run: python3 ./plot_csv.py" << std::endl;
    // system("python3 ./plot_csv.py");

    return 0;
}
