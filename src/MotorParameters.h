#ifndef MOTOR_PARAMETERS_H
#define MOTOR_PARAMETERS_H

#include "ControllerParameters.h"

// Constants obtained from parameters in ELEC3114 Lab 2
#define DC_GAIN             23.8095238095
#define TIME_CONSTANT        0.2293332714

#define COEFF_V             TS * DC_GAIN / (TS + TIME_CONSTANT)
#define COEFF_W_PREV        TIME_CONSTANT / (TS + TIME_CONSTANT)

#endif
