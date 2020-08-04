#include <stdlib.h>
#include <stdio.h>

#include "fix_t.h"
#include "PIDController.h"
#include "Motor.h"

#include "MotorParameters.h"
#include "ControllerParameters.h"

#define ZERO FIX_POINT(0)

volatile fix_t setpointReg, feedbackReg, controlReg;

// PID Controller initialisation
struct pidController pid = {
    .kp = FIX_POINT(KP), .ki = FIX_POINT(KI), .kd = FIX_POINT(KD),
    .setWeightB = FIX_POINT(SW_B), .setWeightC = FIX_POINT(SW_C),
    .filterCoeff = FIX_POINT(N),
    .sampleTime = FIX_POINT(TS), .sampleFreq = FIX_POINT(FS),
    .outputMin = FIX_POINT(OUTPUT_MIN), .outputMax = FIX_POINT(OUTPUT_MAX),

    .intCoeff = FIX_POINT(INT_COEFF),
    .derCoeff1 = FIX_POINT(DER_COEFF1), .derCoeff2 = FIX_POINT(DER_COEFF2),
    
    .setpoint = &setpointReg,
    .feedback = &feedbackReg,
    .controlSignal = &controlReg,

    .integrator = ZERO,
    .differentiator = ZERO,
    .prevError = ZERO
};


// Motor simulator initialisation
struct motor motor = {
    .dcGain = FIX_POINT(DC_GAIN),
    .timeConstant = FIX_POINT(TIME_CONSTANT),

    .wPrev = ZERO,

    .coeffV = FIX_POINT(COEFF_V),
    .coeffWPrev = FIX_POINT(COEFF_W_PREV)
};


int main(void) {

    
    
    return EXIT_SUCCESS;
}
