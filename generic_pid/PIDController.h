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
    const fix_t kp, ki, kd;
    const fix_t setWeightB, setWeightC;
    const fix_t filterCoeff;
    const fix_t sampleTime, sampleFreq;
    const fix_t outputMin, outputMax;

    const fix_t intCoeff;
    const fix_t derCoeff1, derCoeff2;

    volatile fix_t *const setpoint;
    volatile fix_t *const feedback;
    volatile fix_t *const controlSignal;

    fix_t integrator;
    fix_t differentiator;
    fix_t prevError;
};

typedef struct pidController *PIDController;

fix_t runControlAlgorithm(PIDController controller);

#endif
