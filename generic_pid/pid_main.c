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

static void setup();

struct pidController controller = {
    .kp = 1, .ki = 1, .kd = 1,
    .sampleTime = 10, .sampleFreq = 100000,
    .outputMin = -10, .outputMax = 10,

    .setpoint = NULL,
    .feedback = NULL,
    .controlSignal = NULL,

    .iTerm = 0,
    .prevError = 0
};

int main() {
    setup();

    while (1);

    return 0;
}

static void setup() {
    // Setup timer(s)

    // Setup DAC/ADC

    // Setup interrupts

}

void timer_isr() {
    // Disable interrupts
    // output = runControlAlgorithm(&controller, feedback)
    // Reset timer
    // Enable interrupts and exit isr
}
