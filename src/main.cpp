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
  unsigned long tstart = micros();
}