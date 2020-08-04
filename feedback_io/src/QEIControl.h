// QEIControl.h
//
// Written By: Aaron Lucas
// Date Created: 2020/08/02
//
// Quadrature encoder interface and control module for the TM4C123GH6PM
// microcontroller.
//
// Written for the Off-World Robotics Team

#ifndef QEI_CONTROL_H
#define QEI_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

typedef uint32_t microseconds;
typedef uint32_t arcsecPerSec;

enum QEIModule {
    QEI0,
    QEI1
};

struct Angle {
    uint16_t degrees;
    uint8_t  minutes;
    uint8_t  seconds;
};

struct Encoder {
    uint32_t numPulses;
    bool hasIndexSignal;
    bool channelsSwapped;
};

enum QEIFilter {
    QEI_NO_FILTER = 0,
    QEI_FILTER_2  = 2, 
    QEI_FILTER_3, 
    QEI_FILTER_4, 
    QEI_FILTER_5, 
    QEI_FILTER_6, 
    QEI_FILTER_7, 
    QEI_FILTER_8, 
    QEI_FILTER_9, 
    QEI_FILTER_10, 
    QEI_FILTER_11, 
    QEI_FILTER_12, 
    QEI_FILTER_13, 
    QEI_FILTER_14, 
    QEI_FILTER_15, 
    QEI_FILTER_16, 
    QEI_FILTER_17,
};

enum QEIDivider {
    QEI_DIVIDE_1,
    QEI_DIVIDE_2,
    QEI_DIVIDE_4,
    QEI_DIVIDE_8,
    QEI_DIVIDE_16,
    QEI_DIVIDE_32,
    QEI_DIVIDE_64,
    QEI_DIVIDE_128
};

void qeiConfigureForEncoder(enum QEIModule qei, struct Encoder encoder);

void qeiConfigureVelocityCapture(enum QEIModule qei, enum QEIDivider div, microseconds period);

void qeiConfigureInputFilter(enum QEIModule qei, enum QEIFilter filter);

void qeiEnableTimerInterrupt(enum QEIModule qei, void (*handler)(void));

void qeiCalibratePosition(enum QEIModule qei, struct Angle angle);

struct Angle qeiGetPosition(enum QEIModule qei);

arcsecPerSec qeiGetVelocity(enum QEIModule qei);

#endif
