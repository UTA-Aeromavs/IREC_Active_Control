#include <Arduino.h>
#include <Wire.h>

#include "Vmath.h"
#include "Qmath.h"
#include "IMU.h"

ICM20649IMU imu;

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("Testing Serial Communication");
  if(!imu.init()){
    Serial.println("failed to innitialize sensor");
  }
  Vec3 hello;
  hello.x = 1;
  hello.y = 2;
  hello.z = 3;
  Serial.println(hello.norm());
  Vec3 world(1, 1, 1);
  Serial.println(dot(hello, world));

  Quat foo(1, 0, 0, 0);
  Quat bar(0, 0.5, 0.5, 3);
  Serial.print((foo+bar).norm());
}

void loop() {
  // Vec3 acceleration = imu.get_raw_acceleration();
  // Vec3 gyroscope = imu.get_raw_angular_velocity();
  // Serial.print(micros());
  // Serial.print(" : gx");
  // Serial.print(gyroscope.x);
  // Serial.print(" : gy");
  // Serial.print(gyroscope.y);
  // Serial.print(" : gz");
  // Serial.println(gyroscope.z);
  delay(500);
}