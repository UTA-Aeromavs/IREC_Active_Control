#pragma once
#include "QMath.h"

class Vec3{
    public:
        float x;
        float y;
        float z;

        Vec3()=default;
        Vec3(float x, float y, float z);

        Vec3 operator+(const Vec3 &other);
        Vec3 operator-(const Vec3 &other);
        Vec3 operator*(const float &other);
        float norm();
        float normRecip();
        Vec3 normalize();
        Quat euler_to_quat();
};
float dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);