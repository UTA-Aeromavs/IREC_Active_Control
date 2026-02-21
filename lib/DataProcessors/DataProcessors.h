#pragma once
#include "Vmath.h"

/*
This is a Digital low pass filter later I should move this to a new file
*/
class DigitalLowPass{
    private:
        unsigned long time_at_last_change;
        float threshold = 0.04f;
        float fc = 0.1f; // LPF corner frequency, Hz
        Vec3 drift = Vec3(0,0,0);
        bool check_motion(Vec3 derivative, unsigned long timestamp);
    public:
        DigitalLowPass(unsigned long init_time);
        Vec3 filter(Vec3 input_derivative, unsigned long timestamp);
};

template <typename T>
Vec3 static_calibration(T* IMU, Vec3 (T::*sensor_callback)(bool) ,unsigned int samples, unsigned int dt);