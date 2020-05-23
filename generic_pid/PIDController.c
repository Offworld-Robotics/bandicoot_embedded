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
    
    // Calculate difference between setpoint and measurement
    fix_t error = fixSubtract(*(controller->setpoint), *(controller->feedback));
    if (error == fixOverflow) return fixOverflow;

    // Calculate P, I and D controller terms, ensuring overflow does not occur
    // in each operation.

    // Proportional term
    fix_t pTerm = fixMultiply(controller->kp, error);
    if (pTerm == fixOverflow) return fixOverflow;

    // Integral term
    fix_t deltaI = fixMultiply(controller->ki, error);
    if (deltaI == fixOverflow) return fixOverflow;

    deltaI = fixMultiply(deltaI, controller->sampleTime); 
    if (deltaI == fixOverflow) return fixOverflow;

    fix_t iTerm = fixAdd(controller->iTerm, deltaI);
    if (iTerm == fixOverflow) return fixOverflow;

    controller->iTerm = iTerm;

    // Derivative Term
    fix_t errorDiff = fixSubtract(error, controller->prevError);
    if (errorDiff == fixOverflow) return fixOverflow;

    fix_t dTerm = fixMultiply(controller->kd, errorDiff);
    if (dTerm == fixOverflow) return fixOverflow;

    dTerm = fixMultiply(dTerm, controller->sampleFreq);
    if (dTerm == fixOverflow) return fixOverflow;

    // Sum to make total control signal
    fix_t controlSignal = fixAdd(fixAdd(pTerm, iTerm), dTerm);
    if (controlSignal == fixOverflow) return fixOverflow;

    // Saturate control signal if required
    if (controlSignal < controller->outputMin)
        controlSignal = controller->outputMin;
    else if (controlSignal > controller->outputMax)
        controlSignal = controller->outputMax;

    // Set new control signal
    *(controller->controlSignal) = controlSignal;

    // Track previous error term for future derivative calaulation
    controller->prevError = error;

    return controlSignal;
}

