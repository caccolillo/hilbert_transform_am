
#include "delay_line.hpp"

data_type delay_line(data_type filter_in) {
    const int DELAY_SAMPLES = 16;
    static data_type buffer[DELAY_SAMPLES] = {0};
    static int index = 0;
    
    // Get the delayed output (oldest sample in buffer)
    data_type output = buffer[index];
    
    // Store new input sample
    buffer[index] = filter_in;
    
    // Update circular buffer index
    index = (index + 1) % DELAY_SAMPLES;
    
    return output;
}