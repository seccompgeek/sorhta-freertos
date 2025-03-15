#include <stdint.h>

/* Stack configuration */
#define STACK_SIZE 0x10000    /* 64KB of stack space */

__attribute__ ((section(".stack")))
static uint8_t stack[STACK_SIZE] __attribute__((aligned(16)));

/* Forward declarations */
extern int main(void);
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
    extern uint64_t _sdata, _edata, _sbss, _ebss, _etext;
    uint64_t *src, *dest;

    /* Copy the data segment initializers */
    src = &_etext;
    dest = &_sdata;
    while (dest < &_edata) {
        *dest++ = *src++;
    }
    
    /* Zero fill the bss segment */
    dest = &_sbss;
    while (dest < &_ebss) {
        *dest++ = 0;
    }
    
    /* Set up exception vectors */
    uint64_t vbar = (uint64_t)g_pfnVectors;
    __asm__ __volatile__("msr vbar_el1, %0" : : "r" (vbar));
    
    /* Call system initialization function */
    extern void SystemInit(void);
    SystemInit();
    
    /* Call the application's entry point */
    main();
    
    /* Infinite loop */
    while (1);
}

/* Default handler for unhandled interrupts */
void Default_Handler(void)
{
    while (1);
}