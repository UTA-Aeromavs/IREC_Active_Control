#pragma once

class Vec3;

class Quat
{
public:
    float q0;
    float q1;
    float q2;
    float q3;
    
    // Reserve the object
    Quat() = default;
    // Construct the object with default values
    Quat(float q0, float q1, float q2, float q3);

    /*
    Quaternion Multiplication
    To multiply 2 quaternions just write r * s
    */
    Quat operator*(const Quat &other);
    /*
    Quaternion Addition
    To multiply 2 quaternions just write r + s
    */
    Quat operator+(const Quat &other);
    /*
    Quaternion Substraction
    To subtract 2 quaternions just write r - s
    */
    Quat operator-(const Quat &other);
    // Quaternion Scaling
    Quat operator*(const float s);
    Quat conj();
    float norm();
    float normRecip();
    Quat normalize();
    Vec3 quat_to_euler();
    // add conversion to euler angles later
};

Quat rotatePassive(Quat rotation_quaternion, Quat rotated_quaternion);
Quat rotateActive(Quat rotation_quaternion, Quat rotated_quaternion);