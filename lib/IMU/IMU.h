#pragma once
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20649.h>
#include "Vmath.h"
#include "Bframe.h"
#include "DataProcessors.h"


// SCL on icm
#define SENSOR_CLK 7
// AD0 on icm
#define SENSOR_MISO 6
// SDA on icm
#define SENSOR_MOSI 5
#define SENSOR_CS 4

struct IMUReading{
    sensors_event_t accelerometer_reading;
    sensors_event_t gyroscope_reading;
    sensors_event_t temperature_reading;
    unsigned long timestamp;
};


/*
Each of the following objects represents a physical chip
These are wrappers that allow you to 
*/
class ICM20649IMU{
    private:
        Adafruit_ICM20649 controller; 
        Vec3 accelerometer_calibration;
        Vec3 gyroscope_calibration;
        DigitalLowPass *gyro_filter; // otherwise constructor is called
        IMUReading buffer;
    public:
        ICM20649IMU() : gyro_filter(nullptr) {}
        ~ICM20649IMU() { delete gyro_filter; }
        bool init(int cs = SENSOR_CS, int clk = SENSOR_CLK, int miso = SENSOR_MISO, int mosi = SENSOR_MOSI);
        IMUReading read();
        Vec3 get_raw_acceleration(bool refresh_buffer = true);
        Vec3 get_raw_angular_velocity(bool refresh_buffer = true);
        void write_bframe(Bframe* current_frame, Bframe *previous_frame);
};
