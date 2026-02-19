#pragma once
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20649.h>
#include "Vmath.h"
#include "Bframe.h"
#include <functional>


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
This is a object that represents the sensor and moves all the sensor functions to it
*/
class ICM20649IMU{
    private:
        Adafruit_ICM20649 controller; 
        Vec3 accelerometer_calibration;
        Vec3 gyroscope_calibration;
        IMUReading buffer;
        Vec3 calibrate(std::function<Vec3(bool)> func, uint samples, uint dt);
    public:
        bool init(int cs = SENSOR_CS, int clk = SENSOR_CLK, int miso = SENSOR_MISO, int mosi = SENSOR_MOSI);
        IMUReading read();
        Vec3 getAcceleration(bool refresh_buffer = true);
        Vec3 getGyro(bool refresh_buffer = true);
        Bframe buildBframe(Bframe* previous_frame);
};

// do not use outside of me!!
