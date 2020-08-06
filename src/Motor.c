#include "Motor.h"

pid_value calculateAngularVelocity(struct motor *motor, pid_value voltage) {
#ifdef FIX_POINT_PID
    fix_t voltageTerm = fixMultiply(motor->coeffV, voltage);
    fix_t wPrevTerm = fixMultiply(motor->coeffWPrev, motor->wPrev);

    fix_t angularVelocity = fixAdd(voltageTerm, wPrevTerm);
#else
    float angularVelocity = motor->coeffV * voltage + motor->coeffWPrev * motor->wPrev;
#endif
    motor->wPrev = angularVelocity;

    return angularVelocity;
}
