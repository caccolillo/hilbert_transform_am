#include <stdio.h>
#include <stdint.h>
#include "filter_envelope_detector.hpp"


#define STEP_INPUT 3
#define NUM_SAMPLES 256000

#define FS 1000000.0     // Sampling frequency: 1 MHz
#define F0 100.0         // Chirp start frequency: 100 Hz
#define F1 20000.0      // Chirp end frequency: 20 kHz
#define AMPLITUDE 0.1      // Chirp amplitude (±0.1)
#define M_PI 3.14159265358979323846


// Define filter function prototype
extern data_type applyIIRFilter(data_type input);


int main() {
	data_type input = 0;
	data_type output = 0;

    // ============================
    // Step Response Section
    // ============================
    FILE *fp = fopen("step_response.csv", "w");
    if (!fp) {
        perror("Failed to open file");
        return 0;
    }

    fprintf(fp, "Sample,Input,Output\n");

    for (int n = 0; n < NUM_SAMPLES; ++n) {
        input = STEP_INPUT;
        output = applyIIRFilter(input);
        fprintf(fp, "%d,%d,%d\n", n, input , output );
    }

    fclose(fp);
    printf("Step response saved to step_response.csv\n");

    // ============================
    // Chirp Signal Section
    // ============================
    FILE *fp_chirp = fopen("chirp_response.csv", "w");
    if (!fp_chirp) {
        perror("Failed to open chirp_response.csv");
        return 0;
    }

    fprintf(fp_chirp, "Sample,Input,Output\n");

    double T = NUM_SAMPLES / FS;         // Total time duration (in seconds)
    double k = (F1 - F0) / T;            // Chirp rate (Hz/s)

    for (int n = 0; n < NUM_SAMPLES; ++n) {
        double t = n / FS;               // Time in seconds
        double phase = 2.0 * M_PI * (F0 * t + 0.5 * k * t * t);  // Chirp phase
        double chirp_val = AMPLITUDE * sin(phase);              // Floating-point value in [-1,1]

        // Convert to signed integer in range [-1, 1]
        input = (data_type)(chirp_val);
        output = applyIIRFilter(input);

        fprintf(fp_chirp, "%d,%d,%d\n", n, input, output);
    }

    fclose(fp_chirp);
    printf("Chirp response saved to chirp_response.csv\n");

    // Run Python plotting script
    system("python3 ./plot_csv.py");

    return 0;
}
