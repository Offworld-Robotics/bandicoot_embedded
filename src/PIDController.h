/*
 * PIDController.c
 *
 * An implementation of a generic PID controller algorithm meant for embedded
 * use in a microcontroller.
 *
 * Features:
 *      - Constant time and memory complexity
 *      - No dynamic memory allocation
 *      - Independent of the standard library
 *      - No floating point arithmetic or division
 *      - Optimised for cycle count
 *
 * Author: Aaron Lucas
 * Date Created: 2020/04/03
 *
 * Written for the Off-World Robotics Team
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef FIX_POINT_PID
#include "fix_t.h"
typedef fix_t pid_value;
#else
typedef float pid_value;
#endif

struct pidController {
    pid_value kp, ki, kd;
    pid_value setWeightB, setWeightC;
    pid_value filterCoeff;
    pid_value sampleTime, sampleFreq;
    pid_value outputMin, outputMax;

    pid_value intCoeff;
    pid_value derCoeff1, derCoeff2;

    volatile pid_value *setpoint;
    volatile pid_value *feedback;
    volatile pid_value *controlSignal;

    pid_value integrator;
    pid_value differentiator;
    pid_value prevError;
};

typedef struct pidController *PIDController;

pid_value runControlAlgorithm(PIDController pid);

#endif
