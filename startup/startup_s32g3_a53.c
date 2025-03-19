#include <stdint.h>

/* Stack configuration */
#define STACK_SIZE 0x10000    /* 64KB of stack space */

/* Forward declarations */
extern int main(void);
extern void SystemInit(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Exception handlers */
void Sync_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void IRQ_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void FIQ_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void SError_Handler(void) __attribute__ ((weak, alias("Default_Handler")));

/* Vector table for AArch64 - must be aligned to 2048 bytes (0x800) */
__attribute__ ((section(".vectors"), aligned(2048)))
void (* const g_pfnVectors[])(void) = {
    Reset_Handler,
    /* Current EL with SP0 */
    Sync_Handler,    /* Synchronous exception */
    IRQ_Handler,     /* IRQ exception */
    FIQ_Handler,     /* FIQ exception */
    SError_Handler,  /* SError exception */
    
    /* Current EL with SPx */
    Sync_Handler,    /* Synchronous exception */
    IRQ_Handler,     /* IRQ exception */
    FIQ_Handler,     /* FIQ exception */
    SError_Handler,  /* SError exception */
    
    /* Lower EL using AArch64 */
    Sync_Handler,    /* Synchronous exception */
    IRQ_Handler,     /* IRQ exception */
    FIQ_Handler,     /* FIQ exception */
    SError_Handler,  /* SError exception */
    
    /* Lower EL using AArch32 */
    Sync_Handler,    /* Synchronous exception */
    IRQ_Handler,     /* IRQ exception */
    FIQ_Handler,     /* FIQ exception */
    SError_Handler,  /* SError exception */
};

/* Reset handler */
void Reset_Handler(void)
{

    /* Now it's safe to call C functions */
    SystemInit();
    main();
    
    /* Should never return, but just in case */
    while(1);
}

/* Default handler for unhandled interrupts */
void Default_Handler(void)
{
    while (1);
}