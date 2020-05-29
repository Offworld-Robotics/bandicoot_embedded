/*
 * main.c
 *
 * PID algorithm control loop
 *
 * Author: Aaron Lucas
 * Date Created: 2020/04/03
 *
 * Written for the Off-World Robotics Team
 */

#include <stddef.h>

#include "PIDController.h"
#include "ControllerParameters.h"

// Fixed point representation of zero (equivalent to normal 0 but used for
// emphasis that it is a fixed point number).
#define ZERO FIX_POINT(0)

// Default value for action flags where no actions are required.
#define CLEAR_ACTION_FLAGS 0

static void setup();

static void resetIterationTimer();

static void updateSetpoint();
static void updateFeedback();
static void updateControlOuput();

// Action flags
static const uint8_t runControlFlag     = 0x01;    // 0000 0001
static const uint8_t resetFlag          = 0x80;    // 1000 0000

// Global flags to indicate next action
static uint8_t actionFlags = CLEAR_ACTION_FLAGS;

// PID Controller initialisation
struct pidController controller = {
    .kp = FIX_POINT(KP), .ki = FIX_POINT(KI), .kd = FIX_POINT(KD),
    .setWeightB = FIX_POINT(SW_B), .setWeightC = FIX_POINT(SW_C),
    .filterCoeff = FIX_POINT(N),
    .sampleTime = FIX_POINT(TS), .sampleFreq = FIX_POINT(FS),
    .outputMin = FIX_POINT(OUTPUT_MIN), .outputMax = FIX_POINT(OUTPUT_MAX),

    .intCoeff = FIX_POINT(INT_COEFF),
    .derCoeff1 = FIX_POINT(DER_COEFF1), .derCoeff2 = FIX_POINT(DER_COEFF2),
    
    .setpoint = NULL,
    .feedback = NULL,
    .controlSignal = NULL,

    .integrator = ZERO,
    .differentiator = ZERO,
    .prevError = ZERO
};

int main() {
    setup();

    while (1) {
        if (actionFlags & runControlFlag) {
            resetIterationTimer();

            updateSetpoint();
            updateFeedback();
            fix_t output = runControlAlgorithm(&controller);

            if (output == fixOverflow) {
                actionFlags |= resetFlag;
            } else {
                updateControlOuput();
            }

            actionFlags &= ~runControlFlag;
        }

        if (actionFlags & resetFlag) {
            resetIterationTimer();

            *(controller.feedback) = ZERO;
            *(controller.controlSignal) = ZERO;

            controller.integrator = ZERO;
            controller.prevError = ZERO;

            actionFlags = CLEAR_ACTION_FLAGS;
        }
    }

    return 0;
}

static void setup() {
    // Setup timer(s)

    // Setup DAC/ADC

    // Setup interrupts

}

static void resetIterationTimer() {
    // timerCountRegister = 0;
}

static void updateSetpoint() {
    // *(controller.setpoint) = someValue;
}

static void updateFeedback() {
    // *(controller.feedback) = valueFromADC;
}

static void updateControlOuput() {
    // controller.controlOutput is already updated from the
    // runControlAlgorithm() function so only the DAC needs to be run to
    // convert it into a voltage.

}

void timer_isr() {
    actionFlags |= runControlFlag;   
}
