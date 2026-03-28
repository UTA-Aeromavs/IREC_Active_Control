#pragma once
#include "QMath.h"
#include <cstdio>   // for snprintf
#include <cstddef>

/*
It holds 3 members : x, y, z
Can be used to perform vector operations
*/
class Vec3{
    public:
        float x;
        float y;
        float z;

        Vec3()=default;
        Vec3(float x, float y, float z);

        // Vector Addition
        Vec3 operator+(const Vec3 &other);
        // Vector Subtraction
        Vec3 operator-(const Vec3 &other);
        // Vector Scaling
        Vec3 operator*(const float &other);
        Vec3 operator/(const float &other);
        // Returns the magnitude of the vector
        float norm();
        // Returns 1/norm() or the reciprical of the vector magnitude
        float normRecip();
        // Returns a unit vector pointing in the same direction
        Vec3 normalize();
        // Transforms a roll-pitch-yaw vector to a quaternion
        Quat euler_to_quat();
        // Logging helpers
        static const char* header() {
            return "x,y,z";
        }

        int toCSV(char* buffer, size_t size) const {
            return snprintf(buffer, size, "%.6f,%.6f,%.6f", x, y, z);
        }
};
// Returns dot product between two vectors
float dot(Vec3 a, Vec3 b);
// Returns cross product between two vectors
Vec3 cross(Vec3 a, Vec3 b);