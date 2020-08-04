#include "Motor.h"

fix_t calculateAngularVelocity(struct motor *motor, fix_t voltage) {
    fix_t voltageTerm = fixMultiply(motor->coeffV, voltage);
    fix_t wPrevTerm = fixMultiply(motor->coeffWPrev, motor->wPrev);

    fix_t angularVelocity = fixAdd(voltageTerm, wPrevTerm);

    motor->wPrev = angularVelocity;

    return angularVelocity;
}
