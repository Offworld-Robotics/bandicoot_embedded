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
    pwmConfigureOutput(PWM01_B7);
    pwmSetPeriod(PWM00_B6, 100);        // 10 kHz PWM frequency
    /* pwmSetPeriod(PWM01_B7, 100);        // 10 kHz PWM frequency */
    pwmSetDutyCycle(PWM00_B6, 250);     // 25% duty cycle
    pwmSetDutyCycle(PWM01_B7, pwmGetDutyCycle(PWM00_B6));     // test pwmGetDutyCycle
    pwmEnableOutput(PWM00_B6, true);
    pwmEnableOutput(PWM01_B7, true);

    pwmConfigureOutput(PWM14_F0);
    pwmConfigureOutput(PWM15_F1);
    pwmSetPeriod(PWM14_F0, 10);         // 100 kHz PWM frequency
    pwmSetDutyCycle(PWM14_F0, 0);       // All off
    pwmSetDutyCycle(PWM15_F1, 1000);    // All on
    pwmEnableOutput(PWM14_F0, true);
    pwmEnableOutput(PWM15_F1, true);

    while (true);
}

