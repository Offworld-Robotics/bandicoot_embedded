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
    fix_t error = *(controller->setpoint) - *(controller->feedback);

    // Calculate P, I and D controller terms
    fix_t pTerm = fixMultiply(controller->kp, error);

    fix_t deltaI = fixMultiply(controller->ki, error);
    deltaI = fixMultiply(deltaI, controller->sampleTime); 
    controller->iTerm += deltaI;

    fix_t dTerm = fixMultiply(controller->kd, (error - controller->prevError));
    dTerm = fixMultiply(dTerm, controller->sampleFreq);

    // Sum to make total control signal
    fix_t controlSignal = pTerm + controller->iTerm + dTerm;

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

