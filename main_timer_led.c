// Timer CPU0 config Reg and Interruption and onboard LED blink
// L. BAGHLI 11/05/2024

// Included Files
#include "F28x_Project.h"

int Count = 0;
// Function Prototypes
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
// InitGpio();  // Skipped for this example

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
// 200MHz *0.5= 10000000-1
   CpuTimer0Regs.PRD.all = 10000000 - 1;
   // Set pre-scale counter to divide by 1 (SYSCLKOUT):
   CpuTimer0Regs.TPR.all  = 0;
   CpuTimer0Regs.TPRH.all  = 0;
   // Initialize timer control register:
   CpuTimer0Regs.TCR.bit.TSS = 1;     // 1 = Stop timer, 0 = Start/Restart Timer
   CpuTimer0Regs.TCR.bit.TRB = 0;     // 1 = reload timer
   CpuTimer0Regs.TCR.bit.SOFT = 0;
   CpuTimer0Regs.TCR.bit.FREE = 0;    // Timer Free Run Disabled
   CpuTimer0Regs.TCR.bit.TIE = 1;     // 0 = Disable/ 1 = Enable Timer

   CpuTimer0Regs.TCR.bit.TSS = 0;     // 0 = Start/Restart Timer
   // CpuTimer0Regs.TCR.all = 0x4001;   // tout ça équivalent à ça

// Step 5. User specific code, enable __interrupts:
// Configure GPIO34 as a GPIO output pin
   EALLOW;
   GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
   EDIS;

// Enable CPU INT1 which is connected to CPU-Timer 0:
   IER |= M_INT1;

// Enable TINT0 in the PIE: Group 1 __interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global __interrupt INTM
   ERTM;   // Enable Global realtime __interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):
   for(;;);
}

// cpu_timer0_isr - CPU Timer0 ISR that toggles GPIO32 once per 500ms
__interrupt void cpu_timer0_isr(void)
{
   Count++;
   GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
   // Acknowledge this __interrupt to receive more __interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
