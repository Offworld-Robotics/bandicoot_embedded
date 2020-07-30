#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "inc/tm4c123gh6pm.h"

#define SYS_CLOCK_FREQ_MHZ      50

enum Status {
    STATUS_SUCCESS = 0,
    STATUS_FAILURE = 1
};

void setSystemClock(void);

enum Status enablePeripheral(uint32_t peripheral);

#endif
