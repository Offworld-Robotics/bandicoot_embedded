#ifndef MOTOR_SIM_H
#define MOTOR_SIM_H

#ifdef FIX_POINT_PID
#include "fix_t.h"
typedef fix_t pid_value;
#else
typedef float pid_value;
#endif

struct motor {
    pid_value dcGain;
    pid_value timeConstant;

    pid_value wPrev;

    pid_value coeffV;
    pid_value coeffWPrev;
};

pid_value calculateAngularVelocity(struct motor *motor, pid_value voltage);

#endif
