#include "common.h"

#include "driverlib/timer.h"
#include "driverlib/gpio.h"

#ifdef FIX_POINT_PID
#include "fix_t.h"
#define PID_VAL(x)  FIX_POINT(x)
#else
#define PID_VAL(x)  x
#endif

#include "PIDController.h"
#include "Motor.h"
#include "PWMControl.h"

#include "MotorParameters.h"
#include "ControllerParameters.h"

#define ZERO PID_VAL(0)

static void setupTimer(void);
static void setupGPIO(void);
static void setupPWM(void);


static void timer_isr(void);

#ifdef DEBUG
static uint32_t getTimerValue(void);
#endif

volatile pid_value setpointReg, feedbackReg, controlReg;

struct pidController pid;
struct motor motor;

int main(void) {
    setSystemClock();
    enableFPU();

    // PID Controller initialisation
    pid = (struct pidController){
        .kp = PID_VAL(KP), .ki = PID_VAL(KI), .kd = PID_VAL(KD),
        .setWeightB = PID_VAL(SW_B), .setWeightC = PID_VAL(SW_C),
        .filterCoeff = PID_VAL(N),
        .sampleTime = PID_VAL(TS), .sampleFreq = PID_VAL(FS),
        .outputMin = PID_VAL(OUTPUT_MIN), .outputMax = PID_VAL(OUTPUT_MAX),

        .intCoeff = PID_VAL(INT_COEFF),
        .derCoeff1 = PID_VAL(DER_COEFF1), .derCoeff2 = PID_VAL(DER_COEFF2),
        
        .setpoint = &setpointReg,
        .feedback = &feedbackReg,
        .controlSignal = &controlReg,

        .integrator = ZERO,
        .differentiator = ZERO,
        .prevError = ZERO
    };


    // Motor simulator initialisation
    motor = (struct motor){
        .dcGain = PID_VAL(DC_GAIN),
        .timeConstant = PID_VAL(TIME_CONSTANT),

        .wPrev = ZERO,

        .coeffV = PID_VAL(COEFF_V),
        .coeffWPrev = PID_VAL(COEFF_W_PREV)
    };
        
    setupTimer();
    setupGPIO();
    setupPWM();

    setpointReg = PID_VAL(100);

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

static void setupGPIO(void) {
    enablePeripheral(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);
}

static void setupPWM(void) {
    pwmSetClockDivider(PWM_CLKDIV_1);
    pwmConfigureOutput(PWM00_B6);
    pwmSetPeriod(PWM00_B6, 100);
    pwmSetDutyCycle(PWM00_B6, 0);
    pwmEnableOutput(PWM00_B6, true);
}


static void timer_isr(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    feedbackReg = calculateAngularVelocity(&motor, controlReg);
#ifdef FIX_POINT_PID
    if (feedbackReg == fixOverflow || runControlAlgorithm(&pid) == fixOverflow)
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
#else
    runControlAlgorithm(&pid);
#endif

    thousandths duty = controlReg / pid.outputMax * 1000;
    pwmSetDutyCycle(PWM00_B6, duty);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0x00);

}

#ifdef DEBUG
static uint32_t getTimerValue(void) {
    return TimerLoadGet(TIMER0_BASE, TIMER_A);
}
#endif
