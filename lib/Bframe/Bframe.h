#pragma once
#include "Vmath.h"
#include <Arduino.h>

/*
Body Attached Reference Frame
Has 4 members
acceleration - The acceleration experienced by the body
angular_velocity - The velocity experienced by the body
timestamp - The time of measurement
*buffer - A pointer to the previous element
Acts like a linked list which allows for integration and deriviation
You must clear the buffer before memory runs out!
*/
struct Bframe{
    Vec3 acceleration;
    Vec3 angular_velocity;
    unsigned long timestamp;
    Bframe *buffer;
};