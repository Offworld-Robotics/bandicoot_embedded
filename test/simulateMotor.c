#include "common.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"

#include "fix_t.h"
#include "PIDController.h"
#include "Motor.h"

#include "MotorParameters.h"
#include "ControllerParameters.h"

#define ZERO FIX_POINT(0)

static void setupTimer(void);

static void timer_isr(void);

#ifdef DEBUG
static uint32_t getTimerValue(void);
#endif

volatile fix_t setpointReg, feedbackReg, controlReg;

struct pidController pid;
struct motor motor;

int main(void) {
    setSystemClock();
    enableFPU();

    float x = 1.5, y = 2.5;
    float z = x + y;
    z = z + 1;

    // PID Controller initialisation
    pid = (struct pidController){
        .kp = FIX_POINT(KP), .ki = FIX_POINT(KI), .kd = FIX_POINT(KD),
        .setWeightB = FIX_POINT(SW_B), .setWeightC = FIX_POINT(SW_C),
        .filterCoeff = FIX_POINT(N),
        .sampleTime = FIX_POINT(TS), .sampleFreq = FIX_POINT(FS),
        .outputMin = FIX_POINT(OUTPUT_MIN), .outputMax = FIX_POINT(OUTPUT_MAX),

        .intCoeff = FIX_POINT(INT_COEFF),
        .derCoeff1 = FIX_POINT(DER_COEFF1), .derCoeff2 = FIX_POINT(DER_COEFF2),
        
        .setpoint = &setpointReg,
        .feedback = &feedbackReg,
        .controlSignal = &controlReg,

        .integrator = ZERO,
        .differentiator = ZERO,
        .prevError = ZERO
    };


    // Motor simulator initialisation
    motor = (struct motor){
        .dcGain = FIX_POINT(DC_GAIN),
        .timeConstant = FIX_POINT(TIME_CONSTANT),

        .wPrev = ZERO,

        .coeffV = FIX_POINT(COEFF_V),
        .coeffWPrev = FIX_POINT(COEFF_W_PREV)
    };
        
        setupTimer();

    enablePeripheral(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);

    setpointReg = 5;

    while (true);
    
    return 0;
}

static void setupTimer(void) {
    enablePeripheral(SYSCTL_PERIPH_TIMER0);   

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); //| TIMER_CFG_A_ACT_TOINTD);
    TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);

    TimerLoadSet(TIMER0_BASE, TIMER_A, 50000);  // 50000 clock cycles @ 50MHz = 1ms

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, timer_isr);

    TimerEnable(TIMER0_BASE, TIMER_A);
}

static void timer_isr(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    feedbackReg = calculateAngularVelocity(&motor, controlReg);
    runControlAlgorithm(&pid);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00);

}

#ifdef DEBUG
static uint32_t getTimerValue(void) {
    return TimerLoadGet(TIMER0_BASE, TIMER_A);
}
#endif
