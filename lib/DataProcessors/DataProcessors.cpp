#include "DataProcessors.h"
#define _USE_MATH_DEFINES
#include <cmath>

DigitalLowPass::DigitalLowPass(unsigned long init_time){
    time_at_last_change = init_time;
}

bool DigitalLowPass::check_motion(Vec3 derivative, unsigned long timestamp){
    if (derivative.x < threshold 
        && derivative.y < threshold 
        && derivative.z < threshold){
            return true;
    }
    // this is an intentional side effect
    time_at_last_change = timestamp;
    return false;
}

Vec3 DigitalLowPass::filter(Vec3 input, unsigned long timestamp){
    unsigned long dt = timestamp - time_at_last_change;
    if(this->check_motion(input, timestamp) && dt > 1000){
        float alpha = 2*M_PI*fc*(float)(dt) / 1000000;
        drift = drift + (input - drift) * alpha;
    }
    return input - drift;
}