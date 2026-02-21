#include "IMU.h"
#include "Vmath.h"
#include "Bframe.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

IMUReading ICM20649IMU::read()
{
    IMUReading head;
    controller.getEvent(&head.accelerometer_reading,
                        &head.gyroscope_reading,
                        &head.temperature_reading);
    head.timestamp = micros();
    return head;
}

bool ICM20649IMU::init(int cs, int clk, int miso, int mosi)
{
    if (!controller.begin_SPI(cs, clk, miso, mosi))
    {
        return false;
    }
    controller.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
    Serial.print("Accelerometer range set to: ");
    switch (controller.getAccelRange())
    {
    case ICM20649_ACCEL_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case ICM20649_ACCEL_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case ICM20649_ACCEL_RANGE_16_G:
        Serial.println("+-16G");
        break;
    case ICM20649_ACCEL_RANGE_30_G:
        Serial.println("+-30G");
        break;
    }

    controller.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
    Serial.print("Gyro range set to: ");
    switch (controller.getGyroRange())
    {
    case ICM20649_GYRO_RANGE_500_DPS:
        Serial.println("500 degrees/s");
        break;
    case ICM20649_GYRO_RANGE_1000_DPS:
        Serial.println("1000 degrees/s");
        break;
    case ICM20649_GYRO_RANGE_2000_DPS:
        Serial.println("2000 degrees/s");
        break;
    case ICM20649_GYRO_RANGE_4000_DPS:
        Serial.println("4000 degrees/s");
        break;
    }

    controller.setAccelRateDivisor(0);
    uint16_t accel_divisor = controller.getAccelRateDivisor();
    float accel_rate = 1125 / (1.0 + accel_divisor);

    Serial.print("Accelerometer data rate divisor set to: ");
    Serial.println(accel_divisor);
    Serial.print("Accelerometer data rate (Hz) is approximately: ");
    Serial.println(accel_rate);

    controller.setGyroRateDivisor(0);
    uint8_t gyro_divisor = controller.getGyroRateDivisor();
    float gyro_rate = 1100 / (1.0 + gyro_divisor);

    Serial.print("Gyro data rate divisor set to: ");
    Serial.println(gyro_divisor);
    Serial.print("Gyro data rate (Hz) is approximately: ");
    Serial.println(gyro_rate);
    Serial.println();

    Serial.println("Starting Calibration Sequence");
    Serial.println("Calibrating accelerometer");

    accelerometer_calibration = static_calibration(this, &ICM20649IMU::get_raw_acceleration, 5000, 1);
    gyroscope_calibration = static_calibration(this, &ICM20649IMU::get_raw_angular_velocity, 5000, 1);

    gyro_filter = new DigitalLowPass(buffer.timestamp);
    Serial.println("SETUP COMPLETE");
    return true;
}

Vec3 ICM20649IMU::get_raw_acceleration(bool refresh_buffer)
{
    if (refresh_buffer)
    {
        buffer = this->read();
    }
    return Vec3(buffer.accelerometer_reading.acceleration.x,
                buffer.accelerometer_reading.acceleration.y,
                buffer.accelerometer_reading.acceleration.z);
}

Vec3 ICM20649IMU::get_raw_angular_velocity(bool refresh_buffer)
{
    if (refresh_buffer)
    {
        buffer = this->read();
    }
    return Vec3(buffer.gyroscope_reading.gyro.x,
                buffer.gyroscope_reading.gyro.y,
                buffer.gyroscope_reading.gyro.z);
}



void ICM20649IMU::write_bframe(Bframe* current_frame, Bframe *previous_frame)
{
    current_frame->acceleration = this->get_raw_acceleration() - accelerometer_calibration;
    current_frame->angular_velocity = gyro_filter->filter(this->get_raw_angular_velocity(false) - gyroscope_calibration, buffer.timestamp);
    current_frame->timestamp = buffer.timestamp;
    current_frame->buffer = previous_frame;
}

template <typename T>
Vec3 static_calibration(T* IMU, Vec3 (T::*sensor_callback)(bool) ,uint samples, uint dt){
        Vec3 mean(0, 0, 0);
    for (int i = 0; i < samples; i++)
    {
        mean = mean + (IMU->*sensor_callback)(true);
        delay(dt);
    }
    mean = mean * (1 / (float)samples);
    Serial.println("Calibrate Done");
    Serial.print("Offsets : ");
    Serial.print(mean.x);
    Serial.print(", ");
    Serial.print(mean.y);
    Serial.print(", ");
    Serial.println(mean.z);
    return mean;
}