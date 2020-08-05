#ifndef CONTROLLER_PARAMETERS_H
#define CONTROLLER_PARAMETERS_H

// PID Gains
#define KP                  0.058
#define KI                  0.5044
#define KD                  -0.00042527

// Setpoint weights
#define SW_B                0.0411
#define SW_C                0.2304

// Filter coefficient
#define N                   22.0

// Sampling parameters
#define FS                  1000.0
#define TS                  1.0 / FS

// Saturation limits
#define OUTPUT_MIN          -24.0
#define OUTPUT_MAX          24.0

// Internal controller coefficients
#define INT_COEFF           KI * TS
#define DER_COEFF1          KD * N
#define DER_COEFF2          1.0/(1.0 + N*TS)

#endif
