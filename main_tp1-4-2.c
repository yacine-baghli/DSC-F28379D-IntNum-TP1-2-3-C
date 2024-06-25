// Timer CPU0 100us and onboard LED + Carte verte blink
//L. BAGHLI 11/05/2024

// Included Files
#include "F28x_Project.h"

int Count = 0;
int Ncount = 2500-1;
// Function Prototypes
void Gpio_setup();
__interrupt void cpu_timer0_isr(void);

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
   PieVectTable.TIMER0_INT = &cpu_timer0_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize the Device Peripheral. This function can be
//         found in F2837xD_CpuTimers.c
//   InitCpuTimers();   // For this example, only initialize the Cpu Timers
// Configure CPU-Timer 0 to __interrupt every 500 milliseconds:
// 200MHz CPU Freq, 500 millisecond Period (in uSeconds)
//   ConfigCpuTimer(&CpuTimer0, 200, 500000);
// 200MHz *100µs= 20.000-1
   CpuTimer0Regs.PRD.all = 20000 - 1;
   // Set pre-scale counter to divide by 1 (SYSCLKOUT):
   CpuTimer0Regs.TPR.all  = 0;
   CpuTimer0Regs.TPRH.all  = 0;
   // Initialize timer control register:
   CpuTimer0Regs.TIM.all = 0;               // Reset Counter
   CpuTimer0Regs.TCR.bit.TSS = 1;     // 1 = Stop timer, 0 = Start/Restart Timer
   CpuTimer0Regs.TCR.bit.TRB = 0;     // 1 = reload timer
   CpuTimer0Regs.TCR.bit.SOFT = 0;
   CpuTimer0Regs.TCR.bit.FREE = 0;    // Timer Free Run Disabled
   CpuTimer0Regs.TCR.bit.TIE = 1;     // 0 = Disable/ 1 = Enable Timer

   CpuTimer0Regs.TCR.bit.TSS = 0;     // 0 = Start/Restart Timer
   // CpuTimer0Regs.TCR.all = 0x4001;   // tout ça équivalent à ça

// Step 5. User specific code, enable __interrupts:
// Enable CPU INT1 which is connected to CPU-Timer 0:
   IER |= M_INT1;

// Enable TINT0 in the PIE: Group 1 __interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global __interrupt INTM
   ERTM;   // Enable Global realtime __interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   while(true){
       BtnP1();
   }
}

void BtnP1(){
    if(GpioDataRegs.GPADAT.bit.GPIO19){
        GpioDataRegs.GPCSET.bit.GPIO67 = 1;
    }else{
        GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
    }
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
//    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;   // Clear
    GpioDataRegs.GPCDAT.bit.GPIO67 = 1;


    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;     // Disable pull-up on GPIO19
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;    // Configure GPIO19 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;     // GPIO19 = input    OUT2

    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;     // Disable pull-up on GPIO18
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;    // Configure GPIO18 as GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;     // GPIO18 = input    OUT2

    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;       // Onboard LED Output
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;       // Autorisation de write
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

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void){

    Count++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}
