// Exemple Timer0 ISR

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#define EPWM_TIMER_TBPRD    4500     // 90MHz/4500/2 = 10 kHz
int InterruptCount=0;

void Gpio_setup(void);
void InitPWM();
__interrupt void epwm_isr();

void main(void)
{
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2806x_SysCtrl.c file.
    InitSysCtrl();
    Gpio_setup();
    InitPWM();
    
    // Step 2. Initalize GPIO:
    // This example function is found in the F2806x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    // InitGpio();  // Skipped for this example

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags are cleared.
    // This function is found in the F2806x_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2806x_DefaultIsr.c.
    // This function is found in F2806x_PieVect.c.
    InitPieVectTable();

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

// Enable CPU INT3 which is connected to EPWM1-3 INT
    IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-6
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    // Step 6. IDLE loop. Just sit and loop forever (optional)
    //
    for(;;)
    {

    }
}

// epwm_isr
__interrupt void epwm_isr()
{
    GpioDataRegs.GPASET.bit.GPIO12 = 1;    // Set
    InterruptCount++;
    if (InterruptCount&1)  GpioDataRegs.GPASET.bit.GPIO22 = 1;
    else       GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // Clear

    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;  // Clear
}

// InitPWM
void InitPWM()
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    // setup the Time-Base Period Register (TBPRD)
    // 90MHz/4500/2 = 10 kHz
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;     // Set timer period
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;     // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                // Clear counter
    EPwm1Regs.CMPA.half.CMPA = 1000;     // Set compare A value
    EPwm1Regs.CMPB = 1000;               // Set Compare B value
    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up ? down ?
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;  // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;    // Set PWM1B on event B, up count
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;  // Clear PWM1B on event B, down count
    // Interrupt where we will change the Compare Values
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

}

void Gpio_setup(void)
{
    //
    // Enable PWM1-3 on GPIO0-GPIO5
    //
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;   // Enable pullup on GPIO0
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO1
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO2
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO3
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;   // Enable pullup on GPIO4
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO5
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;  // GPIO0 = PWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;  // GPIO1 = PWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // GPIO2 = PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // GPIO3 = PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;  // GPIO4 = PWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;  // GPIO5 = PWM3B
    //
    // Enable I2C-A on GPIO32 - GPIO33
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pullup on GPIO32
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // GPIO32 = SDAA
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pullup on GPIO33
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // GPIO33 = SCLA
    //
    //GPIO12 and 22 output, set it low
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pullup on GPI12
    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1; // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;  // GPIO12 = GPIO12
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;   // GPIO12 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPI22
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1; // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;  // GPI22 = GPI22
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;   // GPI22 = output
    //
    //GPIO28 and GPIO29 input
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 1;   // Disable pullup on GPIO28
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;  // GPIO28 = GPIO28
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 0;   // GPIO28 = output
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;   // Disable pullup on GPIO29
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  // GPIO29 = GPIO29
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 0;   // GPIO29 = output

    EDIS;
}
