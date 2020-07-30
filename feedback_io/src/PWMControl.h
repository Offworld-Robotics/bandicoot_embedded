// PWMControl.h
//
// Written By: Aaron Lucas
// Date Created: 2020/07/29
//
// PWM interface and control module for the TM4C123GH6PM microcontroller.
//
// Written for the Off-World Robotics Team

#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t microseconds;
typedef uint16_t thousandths;  // Used for duty cycle so don't need 32 bits of data

// Collection of all possible PWM outputs and their associated GPIO pins.
// Constants are encoded as PWM[module][signal]_[port][port pin].
// Some PWM signals can be routed to multiple GPIO port pins and such there are
// more PWM outputs than PWM signals that can be generated.
//
// Pairs of PWM signals are created from the same generator block and will share
// settings such as PWM pulse period and clock configuration. Each even-numbered
// PWM signal and its consecutive higher odd-numbered signal are created from the
// same generator and this is indicated by the grouping of PWM outputs.
enum PWMOutput {
    PWM00_B6, PWM01_B7,
    PWM02_B4, PWM03_B5,
    PWM04_E4, PWM05_E5,
    PWM06_C4, PWM06_D0, PWM07_C5, PWM07_D1,

    PWM10_D0, PWM11_D1,
    PWM12_A6, PWM12_E4, PWM13_A7, PWM13_E5,
    PWM14_F0, PWM15_F1,
    PWM16_F2, PWM17_F3
};

// Options for PWM clocking. All PWM modules run off the same clock signal which
// is a divided version of the system clock. The default PWM clock signal is the
// system clock (PWM_CLKDIV_1).
//
// Maximum period for each setting is given by (2^16 - 1)/(f_sysclk / divider).
// For a 50MHz clock, the maxumum periods are given.
enum PWMClockDivider {
    PWM_CLKDIV_1,           //  1.312ms
    PWM_CLKDIV_2,           //  2.621ms
    PWM_CLKDIV_4,           //  5.243ms
    PWM_CLKDIV_8,           // 10.486ms
    PWM_CLKDIV_16,          // 20.971ms
    PWM_CLKDIV_32,          // 41.942ms
    PWM_CLKDIV_64           // 83.885ms
};

// Configure the clock source for all PWM outputs by dividing the system clock
// frequency.
void pwmSetClockDivider(enum PWMClockDivider divider);

// Configure a GPIO pin to become a PWM output.
//
// Enables the correct GPIO port and PWM module peripherals, configures the GPIO
// pin function and configures the PWM generator corresponding to the given PWM
// signal.
void pwmConfigureOutput(enum PWMOutput pwm);

// Set the period of a PWM signal, given in microseconds.
//
// If the given period is not a multiple of the PWM clock period then the
// actual period of the PWM signal will not match exactly due to quantisation
// errors.
//
// Since the PWM period is set in the PWM generators, this function will also
// change the period of the other PWM signal coming from the same generator.
void pwmSetPeriod(enum PWMOutput pwm, microseconds period);

// Get the pulse period of the given PWM output, rounded to the nearest
// microsecond.
microseconds pwmGetPeriod(enum PWMOutput pwm);

// Set the duty cycle of a PWM signal, given in increments of 0.1%.
//
// Rounding errors may cause the actual duty cycle to slightly differ from the
// given duty cycle, with the duration of the high pulse being rounded to the
// nearest multiple of the PWM clock period.
//
// Each PWM output can have its own independent duty cycle, they are not tied to
// generator blocks.
void pwmSetDutyCycle(enum PWMOutput pwm, thousandths duty);

// Get the duty cycle of the given PWM output, rounded to the nearest 0.1%.
// This may be significantly different to the duty cycle that was previously set
// if the PWM period is very low.
thousandths pwmGetDutyCycle(enum PWMOutput pwm);

// Enable or disable the given PWM output.
// Enables the PWM generator if enabling the PWM output and disables the
// generator if both PWM outputs are disabled.
void pwmEnableOutput(enum PWMOutput pwm, bool enable);

#endif
