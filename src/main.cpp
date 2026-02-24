#include <Arduino.h>
#include <Wire.h>

#include "Vmath.h"
#include "Qmath.h"
#include "IMU.h"
#include "Bframe.h"

ICM20649IMU imu;
Bframe *head = new Bframe();
Bframe *getMinusN(Bframe* current, unsigned Nminus);

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
  imu.write_bframe(head, nullptr);
}

void loop() {
  Bframe *prev = head;
  head = new Bframe();
  imu.write_bframe(head, prev);
  Serial.print("a_n:");
  Serial.print(head->acceleration.x);
  if(head->buffer){
    Serial.print(", a_(n-1):");
    Serial.print(head->buffer->acceleration.x);
  }
  if(getMinusN(head, 4)){
    Serial.print(", a_(n-4):");
    Serial.print(getMinusN(head, 4)->acceleration.x);
  }
  Serial.println("");
  // example of cleaning up buffer
  if(getMinusN(head, 4)){
    delete getMinusN(head, 4);
  }
  delay(3000);
}

Bframe *getMinusN(Bframe* current, unsigned Nminus){
  if(Nminus == 0){
    return current;
  }
  if(current == nullptr){
    return nullptr;
  }
  return getMinusN(current->buffer, Nminus-1);
}