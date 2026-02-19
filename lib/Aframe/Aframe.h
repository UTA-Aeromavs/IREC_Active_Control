#pragma once
#include "Vmath.h"
#include "Qmath.h"

struct Aframe{
        Vec3 acceleration;
        Vec3 velocity;
        Vec3 position;
        Vec3 angular_velocity;
        Vec3 orientation;
        unsigned long timestamp;
        Aframe *buffer;
};