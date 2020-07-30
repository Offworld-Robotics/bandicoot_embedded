#include "common.h"

void setSystemClock(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

enum Status enablePeripheral(uint32_t peripheral) {
    // Ensure the peripheral is valid for this device
    if (!SysCtlPeripheralPresent(peripheral))
        return STATUS_FAILURE;

    // Enable peripheral if currently disabled
    if (!SysCtlPeripheralReady(peripheral))
        SysCtlPeripheralEnable(peripheral);

    // Peripherals take 5 clock cycles to become ready after being enabled
    while (!SysCtlPeripheralReady(peripheral));

    return STATUS_SUCCESS;
}
