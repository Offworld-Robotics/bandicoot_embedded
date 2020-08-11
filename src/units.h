// units.h
//
// Written By: Aaron Lucas
// Date Created: 2020/08/06
//
// Definition of commonly used units.
//
// Written for the Off-World Robotics Team.

#ifndef UNITS_H
#define UNITS_H

// Time units
typedef float milliseconds;

// Frequency units
typedef float kilohertz;

// Angular units
typedef float degrees;
typedef float radians;
typedef float rpm;
typedef float radPerSec;

enum RotateDir {
    CLOCKWISE,
    ANTICLOCKWISE
};

struct AngularVel {
    enum RotateDir  direction;
    rpm             speed;
};

// Other units
typedef float percent;

#endif
