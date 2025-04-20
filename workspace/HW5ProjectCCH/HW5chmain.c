//#############################################################################
// FILE:   photoresistor_servo.c
// TITLE:  Photoresistor Controlled Servo
//#############################################################################

#include "F28x_Project.h"
#include "F28379dSerial.h"

#define LAUNCHPAD_CPU_FREQUENCY 200
#define PHOTO_COVERED_THRESHOLD 1.5f  // Adjust based on your photoresistor

// Servo control variables
float servo_angle = 0.0f;
uint16_t adcResult_Photo = 0;
float voltage_Photo = 0.0f;

// Function prototypes
void SetServoAngle(float angle);
__interrupt void ADCA_ISR(void);

void main(void)
{
    // Initialize system
    InitSysCtrl();
    InitGpio();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // Configure serial for debugging
    init_serialSCIA(&SerialA, 115200);

    // Configure GPIO14 as EPWM8A for servo
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;  // Disable pull-up
    EDIS;

    // Configure EPWM8 for 50Hz servo signal
    EPwm8Regs.TBCTL.bit.CTRMODE = 0;     // Up-count mode
    EPwm8Regs.TBCTL.bit.PHSEN = 0;       // Disable phase loading
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = 0;   // High-speed clock divider (/1)
    EPwm8Regs.TBCTL.bit.CLKDIV = 4;      // Clock divider (/16)

    // Period for 50Hz (20ms) = 3.125MHz/50Hz = 62500 counts
    EPwm8Regs.TBPRD = 62500;
    EPwm8Regs.TBPHS.bit.TBPHS = 0;
    EPwm8Regs.TBCTR = 0;

    // Initial position (0° = 1ms pulse)
    EPwm8Regs.CMPA.bit.CMPA = 3125;

    // Action qualifier
    EPwm8Regs.AQCTLA.bit.ZRO = 2;  // Set high at start of period
    EPwm8Regs.AQCTLA.bit.CAU = 1;  // Clear high when CMPA reached

    EALLOW;
    //write configurations for ADCA
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //ADCA

    // Configure SOC0 for ADCINA4
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;    // ADCINA4
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99;   // Sample window = 100 cycles (8µs)
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;  // CPU Timer 0 trigger

    // Configure interrupt
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // Enable INT1
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // Clear INT1 flag
    EDIS;

    // Configure CPU Timer 0 for 10ms interrupts (ADC triggering)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    CpuTimer0Regs.TCR.all = 0x4000;

    // Map ADC interrupt
    EALLOW;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    EDIS;

    // Enable interrupts
    IER |= M_INT1;  // Enable ADC interrupts (Group 1)
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    EINT;
    ERTM;

    // Main loop
    while(1)
    {
        __asm(" NOP");
    }
}

void SetServoAngle(float angle)
{
    // Constrain angle to 0-90 degrees
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 90.0f) angle = 90.0f;

    // Convert angle to pulse width (1ms-2ms range)
    // 0° = 1ms (3125 counts), 90° = 2ms (6250 counts)
    uint16_t pulse_width = 3125 + (uint16_t)((angle/90.0f) * 3125);
    EPwm8Regs.CMPA.bit.CMPA = pulse_width;
}

__interrupt void ADCA_ISR(void)
{
    // Read photoresistor
    adcResult_Photo = AdcaResultRegs.ADCRESULT0;
    voltage_Photo = ((float)adcResult_Photo) * (3.0f / 4096.0f);

    // Control servo based on light level
    if (voltage_Photo < PHOTO_COVERED_THRESHOLD) {
        SetServoAngle(90.0f);  // Move to 90° when covered
    } else {
        SetServoAngle(0.0f);   // Return to 0° when uncovered
    }

    // Debug print
    serial_printf(&SerialA, "Photo: %.2fV -> Servo: %.0f°\r\n",
                  voltage_Photo, (voltage_Photo < PHOTO_COVERED_THRESHOLD) ? 90.0f : 0.0f);

    // Clear interrupt flags
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
