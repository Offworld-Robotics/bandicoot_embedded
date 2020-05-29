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

fix_t runControlAlgorithm(PIDController controller) {
    if (controller == NULL)
        return 0;

    // Temp variable for intermediate calculations
    fix_t temp;

    // Calculate the proportional term
    // -------------------------------------------------------------------------
    
    // Calculate setpoint-weighted error for the proportional term.
    temp = fixMultiply(controller->setWeightB, *(controller->setpoint));
    if (temp == fixOverflow) return fixOverflow;

    fix_t swbError = fixSubtract(temp, *(controller->feedback));
    if (swbError == fixOverflow) return fixOverflow;

    // Proportional term calculation
    fix_t pTerm = fixMultiply(controller->kp, swbError);
    if (pTerm == fixOverflow) return fixOverflow;

    // Calculate the integral term
    // -------------------------------------------------------------------------

    // Error calculation (no setpoint-weighting for integral term)
    fix_t error = fixSubtract(*(controller->setpoint), *(controller->feedback));
    if (error == fixOverflow) return fixOverflow;

    // Integral accumulation
    fix_t deltaI = fixMultiply(controller->intCoeff, error);
    if (deltaI == fixOverflow) return fixOverflow;

    fix_t iTerm = fixAdd(controller->integrator, deltaI);
    if (iTerm == fixOverflow) return fixOverflow;

    // Calculate the derivative term
    // -------------------------------------------------------------------------

    // Calculate setpoint-weighted error for the derivative term.
    temp = fixMultiply(controller->setWeightC, (*controller->setpoint));
    if (temp == fixOverflow) return fixOverflow;

    fix_t swcError = fixSubtract(temp, *(controller->feedback));
    if (swcError == fixOverflow) return fixOverflow;

    // Filtered derivative calculation
    temp = fixSubtract(swcError, controller->prevError);
    if (temp == fixOverflow) return fixOverflow;

    temp = fixMultiply(temp, controller->derCoeff1);
    if (temp == fixOverflow) return fixOverflow;

    temp = fixAdd(temp, controller->differentiator);
    if (temp == fixOverflow) return fixOverflow;

    fix_t dTerm = fixMultiply(temp, controller->derCoeff2);
    if (dTerm == fixOverflow) return fixOverflow;

    // Sum to get control signal
    // -------------------------------------------------------------------------

    fix_t controlSignal = fixAdd(fixAdd(pTerm, iTerm), dTerm);
    if (controlSignal == fixOverflow) return fixOverflow;

    // Saturate control signal if required
    if (controlSignal < controller->outputMin)
        controlSignal = controller->outputMin;
    else if (controlSignal > controller->outputMax)
        controlSignal = controller->outputMax;

    // Update controller states
    // -------------------------------------------------------------------------
    controller->integrator = iTerm;
    controller->differentiator = dTerm;
    controller->prevError = swcError;

    *(controller->controlSignal) = controlSignal;

    return controlSignal;
}

