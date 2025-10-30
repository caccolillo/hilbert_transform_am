
#include "downsampler.hpp"

data_type downsampler(data_type filter_in) {
    static int count = 0;
    static data_type sample = 0;
    
    if (count == 0) {
        sample = filter_in;
    }
    
    count++;
    if (count >= 15) {
        count = 0;
    }
    
    return sample;
}
