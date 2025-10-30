
#include "mean.hpp"

data_type mean(data_type filter_in) {
    const int WINDOW_SIZE = 4;  // Adjust window size as needed
    static data_type buffer[WINDOW_SIZE] = {0};
    static int index = 0;
    static data_type sum = 0;
    static int count = 0;
    
    // Subtract the oldest sample from sum
    sum -= buffer[index];
    
    // Add new sample
    buffer[index] = filter_in;
    sum += filter_in;
    
    // Update circular buffer index
    index = (index + 1) % WINDOW_SIZE;
    
    // Track how many samples we've seen (up to WINDOW_SIZE)
    if (count < WINDOW_SIZE) {
        count++;
    }
    
    return sum / count;
}
