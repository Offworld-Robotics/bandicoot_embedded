// pwmTest.c
//
// Written By: Aaron Lucas
// Date Created: 2020/30/07
//
// Test program for PWMControl.
//
// Written for the Off-World Robotics Team

#include "PWMControl.h"

#include "common.h"

int main(void) {
    setSystemClock();
    pwmSetClockDivider(PWM_CLKDIV_1);
    pwmConfigureOutput(PWM00_B6);
    pwmSetPeriod(PWM00_B6, 100);        // 10 kHz PWM frequency
    pwmSetDutyCycle(PWM00_B6, 250);     // 25% duty cycle
    pwmEnableOutput(PWM00_B6, true);

    while (true);
}

