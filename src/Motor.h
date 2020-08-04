#ifndef MOTOR_SIM_H
#define MOTOR_SIM_H

#include "fix_t.h"

struct motor {
    fix_t dcGain;
    fix_t timeConstant;

    fix_t wPrev;

    fix_t coeffV;
    fix_t coeffWPrev;
};

fix_t calculateAngularVelocity(struct motor *motor, fix_t voltage);

#endif
