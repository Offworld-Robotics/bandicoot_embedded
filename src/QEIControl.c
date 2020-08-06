#include "QEIControl.h"

#include "common.h"
#include "driverlib/qei.h"

#define QEI_CONFIG          QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE

#define TICKS_PER_PULSE     4

#define NUM_QEI_MODULES     2

#define NUM_TICK_DIVIDERS   8

struct QEIModuleData {
    uint32_t numPulses;
    enum QEIDivider divider;
};

static struct QEIModuleData qeiModuleData[NUM_QEI_MODULES] = { 0 };

static const uint32_t qeiBaseAddrs[NUM_QEI_MODULES] = {
    QEI0_BASE,
    QEI1_BASE
};

static const uint32_t qeiTickDividers[NUM_TICK_DIVIDERS] = {
    QEI_VELDIV_1,
    QEI_VELDIV_2,
    QEI_VELDIV_4,
    QEI_VELDIV_8,
    QEI_VELDIV_16,
    QEI_VELDIV_32,
    QEI_VELDIV_64,
    QEI_VELDIV_128
};


#define QEI_BASE(qei)       qeiBaseAddrs[qei]
#define QEI_DATA(qei)       qeiModuleData[qei]
#define QEI_DIVIDER(qei)    qeiTickDividers[qei]

void qeiConfigureForEncoder(enum QEIModule qei, struct Encoder encoder) {
    uint32_t config = QEI_CONFIG;
    config |= encoder.hasIndexSignal ? QEI_CONFIG_RESET_IDX : QEI_CONFIG_NO_RESET;
    config |= encoder.channelsSwapped ? QEI_CONFIG_SWAP : QEI_CONFIG_NO_SWAP;

    QEI_DATA(qei).numPulses = encoder.numPulses;

    QEIConfigure(QEI_BASE(qei), config, encoder.numPulses * TICKS_PER_PULSE);
}

void qeiConfigureVelocityCapture(enum QEIModule qei, enum QEIDivider div, milliseconds period) {
    uint32_t ticks = QEIVelocityConfigure(QEI_BASE(qei), QEI_DIVIDER(qei), 0);
}
