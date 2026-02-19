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
    head.timestamp = millis();
    return head;
}

bool ICM20649IMU::init(int cs, int clk, int miso, int mosi)
{
    if (!controller.begin_SPI(cs, clk, miso, mosi))
    {
        return false;
    }
    // controller.setAccelRange(ICM20649_ACCEL_RANGE_4_G);
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

    // controller.setGyroRange(ICM20649_GYRO_RANGE_500_DPS);
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

    //  controller.setAccelRateDivisor(4095);
    uint16_t accel_divisor = controller.getAccelRateDivisor();
    float accel_rate = 1125 / (1.0 + accel_divisor);

    Serial.print("Accelerometer data rate divisor set to: ");
    Serial.println(accel_divisor);
    Serial.print("Accelerometer data rate (Hz) is approximately: ");
    Serial.println(accel_rate);

    //  controller.setGyroRateDivisor(255);
    uint8_t gyro_divisor = controller.getGyroRateDivisor();
    float gyro_rate = 1100 / (1.0 + gyro_divisor);

    Serial.print("Gyro data rate divisor set to: ");
    Serial.println(gyro_divisor);
    Serial.print("Gyro data rate (Hz) is approximately: ");
    Serial.println(gyro_rate);
    Serial.println();

    Serial.println("Starting Calibration Sequence");
    Serial.println("Calibrating accelerometer");
    // accelerometer_calibration = this->calibrate([this](bool refresh) {return this->getAcceleration(refresh); }, 5000, 1);
    // gyroscope_calibration = this->calibrate([this](bool refresh) {return this->getGyro(refresh); }, 5000, 1);
    accelerometer_calibration = this->calibrate([this](bool refresh) -> Vec3
                                                { return this->getGyro(refresh); }, 5000, 1);
    Serial.println("Calibrating gyroscope");
    gyroscope_calibration = this->calibrate([this](bool refresh) -> Vec3
                                            { return this->getGyro(refresh); }, 5000, 1);
    Serial.println("SETUP COMPLETE");
    return true;
}

Vec3 ICM20649IMU::calibrate(std::function<Vec3(bool)> func, uint samples, uint dt)
{
    Vec3 mean(0, 0, 0);
    for (int i = 0; i < samples; i++)
    {
        Vec3 sensor_reading = func(true);
        mean.x += (float)(sensor_reading.x / samples);
        mean.y += (float)(sensor_reading.y / samples);
        mean.z += (float)(sensor_reading.z / samples);
        delay(dt);
    }
    Serial.println("Calibrate Done");
    Serial.print("Offsets : ");
    Serial.print(mean.x);
    Serial.print(", ");
    Serial.print(mean.y);
    Serial.print(", ");
    Serial.println(mean.z);
    return mean;
}

Vec3 ICM20649IMU::getAcceleration(bool refresh_buffer)
{
    if (refresh_buffer)
    {
        buffer = this->read();
    }
    return Vec3(buffer.accelerometer_reading.acceleration.x,
                buffer.accelerometer_reading.acceleration.y,
                buffer.accelerometer_reading.acceleration.z);
}

Vec3 ICM20649IMU::getGyro(bool refresh_buffer)
{
    if (refresh_buffer)
    {
        buffer = this->read();
    }
    return Vec3(buffer.gyroscope_reading.gyro.x,
                buffer.gyroscope_reading.gyro.y,
                buffer.gyroscope_reading.gyro.z);
}

Bframe ICM20649IMU::buildBframe(Bframe *previous_frame)
{
    Bframe head;
    head.acceleration = this->getAcceleration();
    head.angular_velocity = this->getGyro(false);
    head.timestamp = buffer.timestamp;
    head.buffer = previous_frame;
}