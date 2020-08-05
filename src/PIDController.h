/*
 * PIDController.h
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

#include "fix_t.h"

struct pidController {
    fix_t kp, ki, kd;
    fix_t setWeightB, setWeightC;
    fix_t filterCoeff;
    fix_t sampleTime, sampleFreq;
    fix_t outputMin, outputMax;

    fix_t intCoeff;
    fix_t derCoeff1, derCoeff2;

    volatile fix_t *setpoint;
    volatile fix_t *feedback;
    volatile fix_t *controlSignal;

    fix_t integrator;
    fix_t differentiator;
    fix_t prevError;
};

typedef struct pidController *PIDController;

fix_t runControlAlgorithm(PIDController controller);

#endif
