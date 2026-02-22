#pragma once
#include "Vmath.h"

/*
This library contains various Filters and calibration tools that can be used for a sensor
Built around Vec3 and Quat objects, this module can be tested natively
*/

/*
Runs a A Digital Low Pass (LPF) single pole Infinite Impulse response (IIR) Filter every second while not in motion
*/
class StaticDriftCompensator{
    private:
        unsigned long time_at_last_change;
        float threshold = 0.04f;
        float decay_parameter;
        Vec3 low_pass_drift = Vec3(0,0,0);
        bool check_motion(Vec3 derivative, unsigned long timestamp);
    public:
        /*
        Requires to be initialized with a start time in micros to prevent undefined behavior
        frequency cutoff is approximate frequency cutoff
        */ 
        StaticDriftCompensator(unsigned long init_time, unsigned int frequency_cutoff);

        /*
        Run the filter and return output
        input_derivative must be the derivative of the function, output is new filtered derivative
        give it the timestamp of current measurement
        */
        Vec3 filter(Vec3 input, unsigned long timestamp);
};

/*
Returns static calibration (offset) of instrument
IMU is an instance of any IMU
sensor_callback is a pointer to a sensor callback (must impliment the ability to overwrite internal buffer)
Samples is the number of samples taken during calibration sequence
dt is the number of milliseconds to wait between measurements
Total calibration time is Samples * dt / 1000 seconds
*/
template <typename T>
Vec3 static_calibration(T* IMU, Vec3 (T::*sensor_callback)(bool) ,unsigned int samples, unsigned int dt);