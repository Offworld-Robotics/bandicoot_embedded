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
// Set parameters here
struct pidController controller = {
    .kp = FIX_POINT(1), .ki = FIX_POINT(1), .kd = FIX_POINT(1),
    .sampleTime = FIX_POINT(0.0001), .sampleFreq = FIX_POINT(10000),
    .outputMin = FIX_POINT(-10), .outputMax = FIX_POINT(10),

    .setpoint = NULL,
    .feedback = NULL,
    .controlSignal = NULL,

    .iTerm = ZERO,
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

            controller.iTerm = ZERO;
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
