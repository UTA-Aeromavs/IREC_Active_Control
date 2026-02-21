#pragma once
#include "Vmath.h"
#include "Qmath.h"


/*
Body motion from an Absolute Reference Frame 
Has 7 members
acceleration - The acceleration of the body
velocity - The velocity of the body
position - the position of the body
angular_velocity - The angular velocity of the body
orientation - the orientation of the body
timestamp - The time of measurement
*buffer - A pointer to the previous element
Acts like a linked list which allows for integration and deriviation
You must clear the buffer before memory runs out!
*/
struct Aframe{
        Vec3 acceleration;
        Vec3 velocity;
        Vec3 position;
        Vec3 angular_velocity;
        Vec3 orientation;
        unsigned long timestamp;
        Aframe *buffer;
};