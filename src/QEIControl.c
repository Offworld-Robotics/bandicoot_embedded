#include "QEIControl.h"

#include "common.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include <math.h>

#define QEI_CONFIG          QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE

#define TICKS_PER_PULSE     4

#define NUM_QEI_MODULES     2

#define NUM_QEI_PINS        3       // Number of pins required for a QEI module
#define OPTIONS_PER_PIN     3       // Number of GPIO options per QEI pin
#define NUM_PIN_OPTIONS     NUM_QEI_PINS * OPTIONS_PER_PIN

#define NUM_TICK_DIVIDERS   8

struct QEIModuleData {
    uint32_t            pulsesPerRev;
    kilohertz           sampleFrequency;
    enum QEIDivider     divider;
    enum QEIFilter      filter;

    enum QEIIndexPin    idxPin;
    enum QEIPhaseAPin   phAPin;
    enum QEIPhaseBPin   phBPin;
};

static struct QEIModuleData qeiModuleData[NUM_QEI_MODULES] = {
    { 0, 0.0f, QEI_DIVIDE_1, QEI_NO_FILTER, QEI0_IDX_D3, QEI0_PHA_D6, QEI0_PHB_F1 },
    { 0, 0.0f, QEI_DIVIDE_1, QEI_NO_FILTER, QEI1_IDX_C4, QEI1_PHA_C5, QEI1_PHB_C6 }
};

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

static const uint32_t qeiPeripherals[NUM_QEI_MODULES] = {
    SYSCTL_PERIPH_QEI0,
    SYSCTL_PERIPH_QEI1
};

static const uint32_t gpioPinConfigs[NUM_PIN_OPTIONS] = {
    GPIO_PD3_IDX0,
    GPIO_PF4_IDX0,
    GPIO_PC4_IDX1,

    GPIO_PD6_PHA0,
    GPIO_PF0_PHA0,
    GPIO_PC5_PHA1,

    GPIO_PD7_PHB0,
    GPIO_PF1_PHB0,
    GPIO_PC6_PHB1
};

// Order of GPIO ports for each pin type (IDX, PHA, PHB) is D, F, C, so these
// can be mapped with a modulo 3 operator.
static const uint32_t gpioPeripherals[OPTIONS_PER_PIN] = {
    SYSCTL_PERIPH_GPIOD,
    SYSCTL_PERIPH_GPIOF,
    SYSCTL_PERIPH_GPIOC,
};

static const uint32_t gpioBaseAddrs[OPTIONS_PER_PIN] = {
    GPIO_PORTD_BASE,
    GPIO_PORTF_BASE,
    GPIO_PORTC_BASE,
};

static const uint8_t gpioPins[NUM_PIN_OPTIONS] = {
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_4,

    GPIO_PIN_6,
    GPIO_PIN_0,
    GPIO_PIN_5,

    GPIO_PIN_7,
    GPIO_PIN_1,
    GPIO_PIN_6
};

#define QEI_BASE(qei)               qeiBaseAddrs[qei]
#define QEI_DATA(qei)               qeiModuleData[qei]
#define QEI_DIVIDER(qei)            qeiTickDividers[qei]
#define QEI_PERIPH(qei)             qeiPeripherals[qei]
#define GPIO_PIN_CONFIG(pin)        gpioPinConfigs[pin]
#define GPIO_PERIPH(pin)            gpioPeripherals[pin % 3]
#define GPIO_BASE(pin)              gpioBaseAddrs[pin % 3]
#define GPIO_PIN(pin)               gpioPins[pin]

static void configureGPIOPin(uint32_t pin);

void qeiConfigureModule0Pins(enum QEIIndexPin idx, enum QEIPhaseAPin phA, enum QEIPhaseBPin phB) {
    QEI_DATA(QEI0).idxPin = idx;
    QEI_DATA(QEI0).phAPin = phA;
    QEI_DATA(QEI0).phBPin = phB;
}

void qeiConfigureForEncoder(enum QEIModule qei, struct Encoder encoder) {
    struct QEIModuleData *data = &QEI_DATA(qei);

    uint32_t config = QEI_CONFIG;
    config |= encoder.hasIndexSignal ? QEI_CONFIG_RESET_IDX : QEI_CONFIG_NO_RESET;
    config |= encoder.channelsSwapped ? QEI_CONFIG_SWAP : QEI_CONFIG_NO_SWAP;

    data->pulsesPerRev = encoder.pulsesPerRev;

    // Enable peripherals and setup gpio pins
    enablePeripheral(QEI_PERIPH(qei));

    configureGPIOPin(data->idxPin);
    configureGPIOPin(data->phAPin);
    configureGPIOPin(data->phBPin);

    // Counter should load 1 less than number of ticks when moving in the
    // reverse direction from position 0.
    QEIConfigure(QEI_BASE(qei), config, data->pulsesPerRev * TICKS_PER_PULSE - 1);
}

void qeiConfigureVelocityCapture(enum QEIModule qei, enum QEIDivider div, kilohertz sampleFreq) {
    uint32_t ticks = SYS_CLOCK_FREQ_KHZ / sampleFreq;
    QEIVelocityConfigure(QEI_BASE(qei), QEI_DIVIDER(qei), ticks);

    QEI_DATA(qei).divider = div;
    QEI_DATA(qei).sampleFrequency = sampleFreq;
}

void qeiConfigureInputFilter(enum QEIModule qei, enum QEIFilter filter) {
    if (filter == QEI_NO_FILTER)
        QEIFilterDisable(QEI_BASE(qei));
    else
        QEIFilterConfigure(QEI_BASE(qei), filter);

    QEI_DATA(qei).filter = filter;
}

void qeiEnableTimerInterrupt(enum QEIModule qei, void (*handler)(void)) {
    QEIIntEnable(QEI_BASE(qei), QEI_INTTIMER);
    QEIIntRegister(QEI_BASE(qei), handler);
}

void qeiCalibratePosition(enum QEIModule qei, degrees angle) {
    uint32_t ticks = angle / 360.0f * (QEI_DATA(qei).pulsesPerRev * TICKS_PER_PULSE);
    QEIPositionSet(QEI_BASE(qei), ticks);
}

degrees qeiGetPosition(enum QEIModule qei) {
    uint32_t ticks = QEIPositionGet(QEI_BASE(qei));
    return (float)ticks * 360.0f / (QEI_DATA(qei).pulsesPerRev * TICKS_PER_PULSE);
}

struct AngularVel qeiGetVelocity(enum QEIModule qei) {
    struct QEIModuleData data = QEI_DATA(qei);

    enum RotateDir direction;
    if (QEIDirectionGet(QEI_BASE(qei)) == 1)
        direction = CLOCKWISE;
    else
        direction = ANTICLOCKWISE;
    
    float pulses = (float)(QEIVelocityGet(QEI_BASE(qei)) * 60);
    rpm speed = pulses * data.sampleFrequency * 1000.0f / (data.pulsesPerRev * 2.0f);

    struct AngularVel velocity = {
        .direction = direction,
        .speed = speed
    };

    return velocity;
}   

void qeiEnableModule(enum QEIModule qei, bool enable) {
    if (enable) {
        QEIEnable(QEI_BASE(qei));
        QEIVelocityEnable(QEI_BASE(qei));
    } else {
        QEIDisable(QEI_BASE(qei));
        QEIVelocityDisable(QEI_BASE(qei));
    }
}

static void configureGPIOPin(uint32_t pin) {
    enablePeripheral(GPIO_PERIPH(pin));

    if (pin == QEI0_PHA_F0 || pin == QEI0_PHB_D7)
        GPIOUnlockPin(GPIO_BASE(pin), GPIO_PIN(pin));

    GPIOPinConfigure(GPIO_PIN_CONFIG(pin));
    GPIOPinTypeQEI(GPIO_BASE(pin), GPIO_PIN(pin));
}
