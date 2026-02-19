#pragma once
#include "Vmath.h"
#include <Arduino.h>

struct Bframe{
    Vec3 acceleration;
    Vec3 angular_velocity;
    unsigned long timestamp;
    Bframe *buffer;
};
