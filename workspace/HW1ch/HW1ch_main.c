//#############################################################################
// FILE:   HWstarter_main.c
//
// TITLE:  HW Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

float saturate ( float input, float saturation_limit); // cch creating a function names saturate with arguments "float inputs" and "float saturation_limit"

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
//cch: p8 add in 5 global float variables below
float sinvalue = 0;
float time = 0;
float ampl = 3.0;
float frequency = 0.05;
float offset = 0.25;
int32_t timeint = 0;
float satvalue = 0; // cch: p8 adding another global variable
uint16_t step = 0;  // cch: P10 Keeps track of LED chase sequence

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 200000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 50000); //ch: P5 changed the frequency to 50ms

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            // cch: p8 insert 3 given lines of code below, before serial_printf is called
            timeint = timeint+1;
            time = timeint*0.25;
            sinvalue = ampl*sin(2*PI*frequency*time) + offset;
            satvalue = saturate(sinvalue,2.65);
                // cch: modify/create new serial_printf statement to print outputs for p8
                serial_printf(&SerialA, "Timeint = %ld, Time = %.2f, Input = %.3f, SatOut = %.2f\r\n", timeint,time,sinvalue,satvalue);
				//serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);

              // cch: P4 UARTPrint has to be set to zero to prevent multiple prints until the 50th timer2 interrupt
              // cch: P4 This makes it stay on the periodic rate and not continuous within the while loop
                UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
       // displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

	// Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
	
	
    CpuTimer1.InterruptCount++;
// cch: P10 Led Chase

    switch (step) {
        case 0:
            GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1; // cch: turn off LED 5
            GpioDataRegs.GPASET.bit.GPIO22 = 1;  // Turn on LED 1
            break;
        case 1:
            GpioDataRegs.GPACLEAR.bit.GPIO22 = 1; // Turn off LED 1
            GpioDataRegs.GPCSET.bit.GPIO94 = 1;   // Turn on LED 2
            break;
        case 2:
            GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1; // Turn off LED 2
            GpioDataRegs.GPCSET.bit.GPIO95 = 1;   // Turn on LED 3
            break;
        case 3:
            GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1; // Turn off LED 3
            GpioDataRegs.GPDSET.bit.GPIO97 = 1; // cch: turn on LED 4
            break;
        case 4:
            GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1; // cch: turn off LED 4
            GpioDataRegs.GPDSET.bit.GPIO111 = 1; // cch: turn on LED 5
            break;
        default:
            break;
    }

    step++;  // cch: P10 move to the next LED in the sequence
    if (step > 4) step = 0; // cch: reset step to loop the pattern

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  // cch: acknowledge interrupt


//// cch: P10 create a left arrow when button 3 is pressed (LED: 4,6,7,8,9,10,14)
//    if (GpioDataRegs.GPADAT.bit.GPIO6 == 0) {
//            GpioDataRegs.GPDSET.bit.GPIO97 = 1; // cch: turn on LED 4
//            GpioDataRegs.GPESET.bit.GPIO130 = 1; // cch: turn on LED 6
//            GpioDataRegs.GPESET.bit.GPIO131 = 1; // cch: turn on LED 7
//            GpioDataRegs.GPASET.bit.GPIO25 = 1; // cch: turn on LED 8
//            GpioDataRegs.GPASET.bit.GPIO26 = 1; // cch: turn on LED 9
//            GpioDataRegs.GPASET.bit.GPIO27 = 1; // cch: turn on LED 10
//            GpioDataRegs.GPESET.bit.GPIO158 = 1; // cch: turn on LED 14
//    }
//
//// cch: P10 create a right arrow when button 2 is pressed (LED: 2,6,7,8,9,10,12)
//    else if (GpioDataRegs.GPADAT.bit.GPIO5 == 0) {
//            GpioDataRegs.GPCSET.bit.GPIO94 = 1; // cch: turn on LED 2
//            GpioDataRegs.GPESET.bit.GPIO130 = 1; // cch: turn on LED 6
//            GpioDataRegs.GPESET.bit.GPIO131 = 1; // cch: turn on LED 7
//            GpioDataRegs.GPASET.bit.GPIO25 = 1; // cch: turn on LED 8
//            GpioDataRegs.GPASET.bit.GPIO26 = 1; // cch: turn on LED 9
//            GpioDataRegs.GPASET.bit.GPIO27 = 1; // cch: turn on LED 10
//            GpioDataRegs.GPBSET.bit.GPIO61 = 1; // cch: turn on LED 12
//    }
//
//// cch: P10 turn off LEDs when neither button is pressed
//     else {
//           GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1; // cch: turn off LED 2
//           GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1; // cch: turn off LED 4
//           GpioDataRegs.GPECLEAR.bit.GPIO130 = 1; // cch: turn off LED 6
//           GpioDataRegs.GPECLEAR.bit.GPIO131 = 1; // cch: turn off LED 7
//           GpioDataRegs.GPACLEAR.bit.GPIO25 = 1; // cch: turn off LED 8
//           GpioDataRegs.GPACLEAR.bit.GPIO26 = 1; // cch: turn off LED 9
//           GpioDataRegs.GPACLEAR.bit.GPIO27 = 1; // cch: turn off LED 10
//           GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1; // cch: turn off LED 12
//           GpioDataRegs.GPECLEAR.bit.GPIO158 = 1; // cch: turn off LED 14
//        }


//GpioDataRegs.GPASET.bit.GPIO22 = 1; // cch: turn on LED 1
//GpioDataRegs.GPCSET.bit.GPIO94 = 1; // cch: turn on LED 2
//GpioDataRegs.GPCSET.bit.GPIO95 = 1; // cch: turn on LED 3
//GpioDataRegs.GPDSET.bit.GPIO97 = 1; // cch: turn on LED 4
//GpioDataRegs.GPDSET.bit.GPIO111 = 1; // cch: turn on LED 5
//GpioDataRegs.GPESET.bit.GPIO130 = 1; // cch: turn on LED 6
//GpioDataRegs.GPESET.bit.GPIO131 = 1; // cch: turn on LED 7
//GpioDataRegs.GPASET.bit.GPIO25 = 1; // cch: turn on LED 8
//GpioDataRegs.GPASET.bit.GPIO26 = 1; // cch: turn on LED 9
//GpioDataRegs.GPASET.bit.GPIO27 = 1; // cch: turn on LED 10
//GpioDataRegs.GPBSET.bit.GPIO60 = 1; // cch: turn on LED 11
//GpioDataRegs.GPBSET.bit.GPIO61 = 1; // cch: turn on LED 12
//GpioDataRegs.GPESET.bit.GPIO157 = 1; // cch: turn on LED 13
//GpioDataRegs.GPESET.bit.GPIO158 = 1; // cch: turn on LED 14


}
// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
//// cch: P9 create code to toggle on and off LED10 and 11 every 100ms
//    if ((CpuTimer2.InterruptCount % 2) == 0) { // cch: changed modulus to be every 100ms when frequency is 50ms
//        GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1; // cch: turn on and off LED 10 (GPIO27, GPA)
//        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // cch: turn on and off LED 11 (GPIO60, GPB)
//    }
//
//// cch: P9 push button 1 is pressed then LEDs turn on and off when not pushed
//    if (GpioDataRegs.GPADAT.bit.GPIO4 == 0) {
//        GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1; // cch: turn on and off LED 12 (GPIO61, GPB)
//        GpioDataRegs.GPETOGGLE.bit.GPIO157 = 1; // cch: turn on and off LED 13 (GPIO157, GPE)
//    } else {
//       GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1; // cch: turn LED 12 (GPIO61, GPB)
//       GpioDataRegs.GPECLEAR.bit.GPIO157 = 1; // cch: turn off LED 13 (GPIO157, GPE)
//    }
//
//// cch: P9 push button 4 is pressed then LEDs turn on and off when not pushed
//    if (GpioDataRegs.GPADAT.bit.GPIO7 == 0) {
//            GpioDataRegs.GPETOGGLE.bit.GPIO158 = 1; // cch: turn on and off LED 14 (GPIO158, GPE)
//            GpioDataRegs.GPETOGGLE.bit.GPIO159 = 1; // cch: turn on and off LED 15 (GPIO159, GPE)
//    } else {
//        GpioDataRegs.GPECLEAR.bit.GPIO158 = 1; // cch: turn off LED 14 (GPIO158, GPE)
//        GpioDataRegs.GPECLEAR.bit.GPIO159 = 1; // cch: turn off LED 15 (GPIO159, GPE)
//    }
//
//// cch: P5 changed the interrupt modulus to 5 instead of  50 to get it to print after 5 times of a 50ms timer frequency
//	if ((CpuTimer2.InterruptCount % 5) == 0) {
//		UARTPrint = 1;
//	}
}

float saturate ( float input, float saturation_limit) // cch: creating a function names saturate with parameters "float inputs" and "float saturation_limit"
{
    if (input > saturation_limit) {
        return saturation_limit;
    }

    if (input < -saturation_limit) {
        return -saturation_limit;
    }
    return input;
}
