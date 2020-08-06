#ifndef CONTROLLER_PARAMETERS_H
#define CONTROLLER_PARAMETERS_H

// Very slow response
// ==================

// PID Gains
#define KP                  2.7542E-5f
#define KI                  0.0551f
#define KD                  0.0f

// Setpoint weights
#define SW_B                1.0f
#define SW_C                1.0f

// Filter coefficient
#define N                   100.0f

// Sampling frequency
#define FS                  1000.0f

// Saturation limits
#define OUTPUT_MIN          -24.0f
#define OUTPUT_MAX          24.0f


// Faster response
// ===============

/* // PID Gains */
/* #define KP                  0.058f */
/* #define KI                  0.5044f */
/* #define KD                  -0.00042527f */

/* // Setpoint weights */
/* #define SW_B                0.0411f */
/* #define SW_C                0.2304f */

/* // Filter coefficient */
/* #define N                   22.0f */

/* // Sampling frequency */
/* #define FS                  1000.0f */

/* // Saturation limits */
/* #define OUTPUT_MIN          -24.0f */
/* #define OUTPUT_MAX          24.0f */

// Internal controller coefficients
#define INT_COEFF           KI * TS
#define DER_COEFF1          KD * N
#define DER_COEFF2          1.0f/(1.0f + N * TS)

// Sample time
#define TS                  1.0f / FS

#endif
