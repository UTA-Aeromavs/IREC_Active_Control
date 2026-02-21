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

/*
This object represents an IMU reading, it has 4 members:
accelerometer_reading, gyroscope_reading, temperature_reading, and timestamp
The first 3 members record the sampled value from the sensor, the last the time in micros()
*/
struct IMUReading{
    sensors_event_t accelerometer_reading;
    sensors_event_t gyroscope_reading;
    sensors_event_t temperature_reading;
    unsigned long timestamp;
};

/*
Each of the following objects represents a physical chip
These are wrappers known as a hardware abstraction layer HAL
This allows us to interface with the chips with code that does not rely on Arduino Platform 
*/

/*
ICM 20649 6DoF Imu
https://learn.adafruit.com/adafruit-icm20649-wide-range-6-dof-imu-accelerometer-and-gyro/overview
*/
class ICM20649IMU{
    private:
        Adafruit_ICM20649 controller; 
        Vec3 accelerometer_calibration;
        Vec3 gyroscope_calibration;
        DigitalLowPass *gyro_filter; // otherwise constructor is called
        IMUReading buffer;
    public:
        //These are the constructors and destructors that make sure there is no memory leak
        ICM20649IMU() : gyro_filter(nullptr) {}
        ~ICM20649IMU() { delete gyro_filter; }

        /*
        Initialized the microcontroller.
        Called during setup() of the arduino event loop
        Optional : can be reconfigured with new spi pins
        */
        bool init(int cs = SENSOR_CS, int clk = SENSOR_CLK, int miso = SENSOR_MISO, int mosi = SENSOR_MOSI);
        
        /*
        Reads the data buffer
        Not recommended for use in any code intended to run natively as IMUReading stores the data in a proprietary datatype defined in Adafruit_Sensor.h
        May be used in Arduino code for debugging
        */
        IMUReading read();
        
        /*
        Returns the Raw Acceleration from the ICM20649 Accelerometer
        For data analysis use bframes defined in write_bframe instead
        refresh_buffer takes a new sample overwriting the existing measurement and timestamp stored in buffer. Setting to false simply reads prexisting buffer from last measurement
        */
        Vec3 get_raw_acceleration(bool refresh_buffer = true);

        /*
        Returns the Raw Angular Velocity from the ICM20649 Gyroscope
        For data analysis use bframes defined in write_bframe instead
        refresh_buffer takes a new sample overwriting the existing measurement and timestamp stored in buffer. Setting to false simply reads prexisting buffer from last measurement
        */
        Vec3 get_raw_angular_velocity(bool refresh_buffer = true);

        /*
        Creates a new Bframe
        current_frame takes a pointer to the bframe object to be written to
        links current_frame with previous_frame argument, for fresh buffer simply use a nullptr
        */
        void write_bframe(Bframe* current_frame, Bframe *previous_frame);
};
