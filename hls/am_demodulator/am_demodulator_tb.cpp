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


/*
 * Single-sample-per-call AM Demodulator Testbench (AXIS)
 *
 * Matches am_demodulator() that consumes one input token per call.
 * For each input sample:
 *   - write one AXI-Stream packet
 *   - call am_demodulator()
 *   - drain output_stream while available
 *
 * Uses bit-preserving pack/unpack (memcpy) to move between data_type and ap_int.
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

#define INITIAL_DELAY 436

// Test parameters
const int NUM_SAMPLES = 60000; // increase to 60000 for longer tests
const float SAMPLE_RATE = 480000.0f;
const float CARRIER_FREQ = 100000.0f;
const float MESSAGE_FREQ = 1000.0f;
const float MODULATION_INDEX = 0.8f;

// Helper: bit-preserving pack/unpack (works for ap_fixed / float / ap_int bit-patterns)
static inline ap_int<DataWordSize> pack_to_apint(const data_type &v) {
    ap_int<DataWordSize> out;
    // Zero whole object to avoid garbage bytes
    std::memset(&out, 0, sizeof(out));
    // Copy the minimum number of bytes available (best-effort)
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
    std::cout << "=== AM demod TB (single-sample-per-call) ===" << std::endl;
    std::cout << "Samples: " << NUM_SAMPLES << ", Fs: " << SAMPLE_RATE << " Hz" << std::endl;

    float max_env = 1.0f + MODULATION_INDEX;
    float norm = 1.0f / max_env;

    hls::stream<axis_data> input_stream("input_stream");
    hls::stream<axis_data> output_stream("output_stream");

    std::vector<float> output_buffer; output_buffer.reserve(NUM_SAMPLES / DOWNSAMPLE_FACTOR + 16);
    std::vector<float> expected_by_input_idx(NUM_SAMPLES);

    // Prepare expected message per input sample (baseband scaled)
    for (int i=0;i<NUM_SAMPLES;i++){
        float t = (float)i / SAMPLE_RATE;
        float message = cosf(2.0f * M_PI * MESSAGE_FREQ * t);
        expected_by_input_idx[i] = message * MODULATION_INDEX * norm;
    }

    std::ofstream input_file("am_input.dat");
    std::ofstream output_file("am_output.dat");
    std::ofstream message_file("message_signal.dat");

    if (!input_file || !output_file || !message_file) {
        std::cerr << "Error opening output files\n"; return -1;
    }

    int test_passed = 0, test_failed = 0;
    float max_error = 0.0f; double sum_sq_err = 0.0;
    int produced_outputs = 0; // counts outputs read (used to align to input indices)

    // MAIN LOOP: write one sample, call DUT, drain outputs
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        float t = (float)i / SAMPLE_RATE;
        float message = cosf(2.0f * M_PI * MESSAGE_FREQ * t);
        float am_signal = (1.0f + MODULATION_INDEX * message) * cosf(2.0f * M_PI * CARRIER_FREQ * t) * norm;

        // pack input
        data_type in_fixed = static_cast<data_type>(am_signal);
        axis_data pkt;
        pkt.data = pack_to_apint(in_fixed);
        pkt.keep = -1;
        pkt.last = (i == NUM_SAMPLES-1) ? 1 : 0;

        // write single sample
        input_stream.write(pkt);

        // Save input & expected for plotting
        input_file << i << " " << am_signal << "\n";
        message_file << i << " " << expected_by_input_idx[i] << "\n";

        // call DUT once (consumes one input token)
        am_demodulator(input_stream, output_stream);

        // drain any available outputs produced by this (or previous) calls
        while (!output_stream.empty()) {
            axis_data outpkt = output_stream.read();
            ap_int<DataWordSize> raw = outpkt.data;
            data_type out_ap = unpack_from_apint(raw);

            float out_float;
            #ifdef DEBUG
                out_float = static_cast<float>(out_ap);
            #else
                // If data_type is ap_fixed, prefer .to_float() when available
                #ifdef HLS_AP_FIXED
                    out_float = out_ap.to_float();
                #else
                    out_float = static_cast<float>(out_ap);
                #endif
            #endif

            output_buffer.push_back(out_float);

            // Align output to input index:
            // output number k corresponds to input index = k*DOWNSAMPLE + INITIAL_DELAY
            int corresponding_input_idx = produced_outputs * DOWNSAMPLE_FACTOR + INITIAL_DELAY;

            if (corresponding_input_idx >= 0 && corresponding_input_idx < NUM_SAMPLES) {
                float expected = expected_by_input_idx[corresponding_input_idx];
                float err = fabsf(out_float - expected);
                if (err > max_error) max_error = err;
                sum_sq_err += double(err) * double(err);
                if (err < 0.3f) test_passed++;
                else test_failed++;
            } else {
                // If out-of-range (initial pipeline outputs / tail), skip pass/fail counting
            }

            ++produced_outputs;
        }

        // optional progress
        if ((i+1) % 1000 == 0) std::cout << "Processed " << (i+1) << " samples\n";
    }

    // After main loop, ensure there are no leftover unread items in input_stream
    if (!input_stream.empty()) {
        std::cerr << "WARNING: input_stream not empty after main loop! leftover tokens: (should be 0)\n";
        // Try to flush the DUT until input_stream is empty (safeguard)
        // This will call the DUT repeatedly with no new inputs; if DUT returns immediately when input empty, no effect.
        int guard = 0;
        while (!input_stream.empty() && guard < 10000) {
            am_demodulator(input_stream, output_stream);
            // drain outputs if any
            while (!output_stream.empty()) {
                axis_data outpkt = output_stream.read();
                ap_int<DataWordSize> raw = outpkt.data;
                data_type out_ap = unpack_from_apint(raw);
                float out_float = static_cast<float>(out_ap);
                output_buffer.push_back(out_float);
                ++produced_outputs;
            }
            guard++;
        }
        if (!input_stream.empty()) {
            std::cerr << "ERROR: input_stream still contains tokens - DUT didn't consume them.\n";
        } else {
            std::cerr << "input_stream drained by guard calls\n";
        }
    }

    // Drain any remaining outputs after all inputs
    while (!output_stream.empty()) {
        axis_data outpkt = output_stream.read();
        ap_int<DataWordSize> raw = outpkt.data;
        data_type out_ap = unpack_from_apint(raw);
        float out_float = static_cast<float>(out_ap);
        output_buffer.push_back(out_float);

        int corresponding_input_idx = produced_outputs * DOWNSAMPLE_FACTOR + INITIAL_DELAY;
        if (corresponding_input_idx >= 0 && corresponding_input_idx < NUM_SAMPLES) {
            float expected = expected_by_input_idx[corresponding_input_idx];
            float err = fabsf(out_float - expected);
            if (err > max_error) max_error = err;
            sum_sq_err += double(err)*double(err);
            if (err < 0.3f) test_passed++; else test_failed++;
        }

        ++produced_outputs;
    }

    // Write outputs to file aligned to input indices (prepend INITIAL_DELAY zeros)
    for (int i = 0; i < INITIAL_DELAY; ++i) {
        output_file << i << " " << 0.0f << "\n";
    }
    for (int k = 0; k < (int)output_buffer.size(); ++k) {
        int in_idx = k * DOWNSAMPLE_FACTOR + INITIAL_DELAY;
        output_file << in_idx << " " << output_buffer[k] << "\n";
    }

    input_file.close();
    output_file.close();
    message_file.close();

    int total_checked = test_passed + test_failed;
    float mse = (total_checked > 0) ? float(sum_sq_err / total_checked) : 0.0f;
    float rmse = sqrtf(mse);

    std::cout << "\n=== RESULTS ===\n";
    std::cout << "Outputs produced: " << output_buffer.size() << "\n";
    std::cout << "Checked: " << total_checked << "\n";
    if (total_checked>0) {
        std::cout << "Passed: " << test_passed << " (" << (100.0*test_passed/total_checked) << "%)\n";
        std::cout << "Failed: " << test_failed << " (" << (100.0*test_failed/total_checked) << "%)\n";
    }
    std::cout << "Max err: " << max_error << ", RMSE: " << rmse << "\n";

    bool overall_pass = (total_checked>0) && (test_failed < total_checked * 0.1f) && (max_error < 0.5f);
    if (overall_pass) std::cout << "*** TEST PASSED ***\n";
    else std::cout << "*** TEST FAILED ***\n";

    std::cout << "Output files: am_input.dat, am_output.dat, message_signal.dat\n";

    // Run Python plotting script
    system("python3 ./plot_csv.py");

    return 0;
}
