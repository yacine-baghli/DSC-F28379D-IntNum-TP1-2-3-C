// ePWM1 Interrupt and onboard LED + Carte verte blink
// L. BAGHLI 11/05/2024

// Included Files
#include "F28x_Project.h"

// attention : Si on change PWM TBPRD, il faut aussi changer Kdth
// 200MHz/2= 100 MHz max que peut supporter PWM module ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;
// 100MHz/5000/2 = 10 kHz
#define EPWM_TIMER_TBPRD    5000      // 10 kHz
#define FullPWM 5000
#define HalfPWM 2500
#define EPWM_MAX_DB   0 //20  DB done by DRV8305

// Variables
int Count = 0;
int cmpr = 800;

// Function Prototypes
void Gpio_setup();
void initEPWM();
__interrupt void epwm_isr(void);

void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
   Gpio_setup();
   initEPWM();

// Step 3. Clear all __interrupts and initialize PIE vector table:
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU __interrupts and clear all CPU __interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.EPWM1_INT = &epwm_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. Here PWM


// Step 5. User specific code, enable __interrupts:
//  Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;
// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global __interrupt INTM
   ERTM;   // Enable Global realtime __interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;);
}

void Gpio_setup()
{
    // Enable PWM1-3 on GPIO0-GPIO5, I2C, BP, OUT1
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1;     // Disable pull-up on GPIO32
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;    // Configure GPIO32 as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;     // GPIO32 = input car emplacement du 69M ADCINA6 sur J1.02 P32

    GpioCtrlRegs.GPDPUD.bit.GPIO111 = 1;     // Disable pull-up on GPIO111
    GpioCtrlRegs.GPDMUX1.bit.GPIO111 = 0;    // Configure GPIO111 as GPIO
    GpioCtrlRegs.GPDDIR.bit.GPIO111 = 0;     // GPIO111 = input car emplacement du 69M ADCINB6 sur J1.06 P11

    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;     // Disable pull-up on GPIO22
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    // Configure GPIO22 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;     // GPIO22 = output    OUT2
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   // Clear

    GpioCtrlRegs.GPCPUD.bit.GPIO67 = 1;     // Disable pull-up on GPIO67
    GpioCtrlRegs.GPCMUX1.bit.GPIO67 = 0;    // Configure GPIO67 as GPIO
    GpioCtrlRegs.GPCDIR.bit.GPIO67 = 1;     // GPIO67 = output    OUT1
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;   // Clear

    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;     // Disable pull-up on GPIO19
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    // Configure GPIO19 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;     // GPIO19 = input    OUT2

    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;     // Disable pull-up on GPIO18
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;    // Configure GPIO18 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;     // GPIO18 = input    OUT2

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;       // Onboard LED Output
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPADAT.bit.GPIO31 = 1;       // Onboard LED Off
    GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

    // Enable I2C-A on J1.09 GPIO105 SCL_OLED et J1.10 GPIO104 SDA_OLED
    GpioCtrlRegs.GPDPUD.bit.GPIO105 = 0;   // Enable pullup on GPIO32
    GpioCtrlRegs.GPDMUX1.bit.GPIO105 = 1;  // GPIO105 = SDAA
    GpioCtrlRegs.GPDQSEL1.bit.GPIO105 = 3; // Asynch input
    GpioCtrlRegs.GPDPUD.bit.GPIO104 = 0;   // Enable pullup on GPIO33
    GpioCtrlRegs.GPDQSEL1.bit.GPIO104 = 3; // Asynch input
    GpioCtrlRegs.GPDMUX1.bit.GPIO104 = 1;  // GPIO104 = SCLA
    EDIS;
}

void initEPWM()
{
    EALLOW;
    // Configure the prescaler to the ePWM modules.  Max ePWM input clock is 100 MHz.
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;     // EPWMCLK divider from PLLSYSCLK.  0=/1, 1=/2
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;
    // setup the Time-Base Period Register (TBPRD)
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;     // Set timer period
    EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;     // Phase is 0
    EPwm1Regs.TBCTR = 0x0000;                // Clear counter
    EPwm1Regs.CMPA.bit.CMPA = cmpr;     // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = cmpr;               // Set Compare B value
    // Setup counter mode
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // Setup shadowing
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // Set actions
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;  // Set PWM1A on event A, up count
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;    // Clear PWM1A on event A, down count
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

// epwm_isr
__interrupt void epwm_isr()
{
    GpioDataRegs.GPCSET.bit.GPIO67 = 1;
    Count++;
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    if (Count&1)  GpioDataRegs.GPASET.bit.GPIO22 = 1;
    else       GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // Clear

//    th += dth;
//    if (th > 2 * M_PI) th -= 2 * M_PI;
//    a = Half + Half * sin(th);
//    b = Half + Half * sin(th - 0.6667 * M_PI);
//    c = Half + Half * sin(th + 0.6667 * M_PI);
//    EPwm1Regs.CMPA.half.CMPA = a;     // Set compare A value
//    EPwm1Regs.CMPB = a;
//    EPwm2Regs.CMPA.half.CMPA = b;     // Set compare A value
//    EPwm2Regs.CMPB = b;               // Set Compare B value
//    EPwm3Regs.CMPA.half.CMPA = c;     // Set compare A value
//    EPwm3Regs.CMPB = c;               // Set Compare B value

    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
}

