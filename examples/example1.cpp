#include <Arduino.h>
#include <Wire.h>

#include "Vmath.h"
#include "Qmath.h"
#include "IMU.h"
#include "Bframe.h"

// This is the icm imu defined in global
ICM20649IMU imu;
// When defining a pointer ALWAYS define it in the global scope
// If you define it in a function like setup() it will become a dangling pointer as soon as the the function frame is dropped off the call stack
Bframe *head = new Bframe();
// This is an example of a recursive function that can be used to easily parse the buffer
// Unlike in arduino ino, in cpp functions must be declared before they are used. The implementation however can be placed anywhere as seen by this example
Bframe *getMinusN(Bframe* current, unsigned Nminus);

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("Testing Serial Communication");
  // init() is a bool so at can be used like this
  if(!imu.init()){
    Serial.println("failed to innitialize sensor");
  }
  // Vectors and quarts can either be declared implicitly . . .
  Vec3 hello;
  hello.x = 1;
  hello.y = 2;
  hello.z = 3;
  // To operate on a vector just call a function from it
  Serial.println(hello.norm());
  //. . . or implicitly
  Vec3 world(1, 1, 1);
  // Some functions like dot product operate on multiple variables so they can be taken out
  Serial.println(dot(hello, world));
  // Quaternions can be defined just like vectors
  Quat foo(1, 0, 0, 0);
  Quat bar(0, 0.5, 0.5, 3);
  // And for both vectors and quaternions can use + - * / as per the rules of linear algebra
  // Order of operations matters
  Serial.print((foo+bar).norm());
  // For the first bframe we write, bind its buffer to nullptr
  // if we run a logical check on nullptr, it is resolved as false which is used later
  imu.write_bframe(head, nullptr);
}

void loop() {
  // Here we pass the pointer to another so we can create make room for a new measurement
  Bframe *prev = head;
  head = new Bframe();
  // write_bframe pulls double duty: it takes a measurement from the imu and chains together the prevous bframe into a linked list
  imu.write_bframe(head, prev);
  // To use values from the bframe use -> syntax. It is the shorthand for dereferencing the pointer like (*ptr).attribute
  Serial.print("a_n:");
  Serial.print(head->acceleration.x);
  // Since we have a linked list you can access previous elements for taking derivatives and integrals
  // First we check that the previous element exists (if its a nullptr, this will return false and skip)
  if(head->buffer){
    // Then we can read the value from the bframe in the buffer like this
    // Since c++ reads left to right, you can chain as many buffers and methods as you would like!
    Serial.print(", a_(n-1):");
    Serial.print(head->buffer->acceleration.x);
  }
  // For arbitrarily long buffers, we can access element n-i using recursion.
  if(getMinusN(head, 4)){
    Serial.print(", a_(n-4):");
    Serial.print(getMinusN(head, 4)->acceleration.x);
  }
  Serial.println("");
  // Before you add a new element be sure to delete the tail of the linked list - otherwise the memory will explode
  if(getMinusN(head, 4)){
    delete getMinusN(head, 4);
  }
  delay(3000);
}

// This is a simple recursive function that can operate on a Bframe
// We can make it work on both bframes and Aframes using template feature in C++ but as that is quite advanced, use this function for illistration purposes
Bframe *getMinusN(Bframe* current, unsigned Nminus){
  // Return the current Bframe if it is zero
  if(Nminus == 0){
    return current;
  }
  // Return null if at the end of the stack
  if(current == nullptr){
    return nullptr;
  }
  // recurse
  return getMinusN(current->buffer, Nminus-1);
}