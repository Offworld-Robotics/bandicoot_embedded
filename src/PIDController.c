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

#include <stddef.h>

#include "PIDController.h"

pid_value runControlAlgorithm(PIDController pid) {
    if (pid == NULL)
        return 0;

#ifdef FIX_POINT_PID
    // Temp variable for intermediate calculations
    fix_t temp;

    // Calculate the proportional term
    // -------------------------------------------------------------------------
    
    // Calculate setpoint-weighted error for the proportional term.
    temp = fixMultiply(pid->setWeightB, *(pid->setpoint));
    if (temp == fixOverflow) return fixOverflow;

    fix_t swbError = fixSubtract(temp, *(pid->feedback));
    if (swbError == fixOverflow) return fixOverflow;

    // Proportional term calculation
    fix_t pTerm = fixMultiply(pid->kp, swbError);
    if (pTerm == fixOverflow) return fixOverflow;

    // Calculate the integral term
    // -------------------------------------------------------------------------

    // Error calculation (no setpoint-weighting for integral term)
    fix_t error = fixSubtract(*(pid->setpoint), *(pid->feedback));
    if (error == fixOverflow) return fixOverflow;

    // Integral accumulation
    fix_t deltaI = fixMultiply(pid->intCoeff, error);
    if (deltaI == fixOverflow) return fixOverflow;

    fix_t iTerm = fixAdd(pid->integrator, deltaI);
    if (iTerm == fixOverflow) return fixOverflow;

    // Calculate the derivative term
    // -------------------------------------------------------------------------

    // Calculate setpoint-weighted error for the derivative term.
    temp = fixMultiply(pid->setWeightC, (*pid->setpoint));
    if (temp == fixOverflow) return fixOverflow;

    fix_t swcError = fixSubtract(temp, *(pid->feedback));
    if (swcError == fixOverflow) return fixOverflow;

    // Filtered derivative calculation
    temp = fixSubtract(swcError, pid->prevError);
    if (temp == fixOverflow) return fixOverflow;

    temp = fixMultiply(temp, pid->derCoeff1);
    if (temp == fixOverflow) return fixOverflow;

    temp = fixAdd(temp, pid->differentiator);
    if (temp == fixOverflow) return fixOverflow;

    fix_t dTerm = fixMultiply(temp, pid->derCoeff2);
    if (dTerm == fixOverflow) return fixOverflow;

    // Sum to get control signal
    // -------------------------------------------------------------------------

    fix_t controlSignal = fixAdd(fixAdd(pTerm, iTerm), dTerm);
    if (controlSignal == fixOverflow) return fixOverflow;

#else
    float pTerm = pid->kp * (pid->setWeightB * *(pid->setpoint) - *(pid->feedback));
    float iTerm = pid->intCoeff * (*(pid->setpoint) - *(pid->feedback)) + pid->integrator;
    float swcError = pid->setWeightC * *(pid->setpoint) - *(pid->feedback);
    float dTerm = pid->derCoeff2 * (pid->differentiator + pid->derCoeff1 * (swcError - pid->prevError));

    float controlSignal = pTerm + iTerm + dTerm;

#endif

    // Saturate control signal if required
    if (controlSignal < pid->outputMin)
        controlSignal = pid->outputMin;
    else if (controlSignal > pid->outputMax)
        controlSignal = pid->outputMax;

    // Update pid states
    // -------------------------------------------------------------------------
    pid->integrator = iTerm;
    pid->differentiator = dTerm;
    pid->prevError = swcError;

    *(pid->controlSignal) = controlSignal;

    return controlSignal;
}

