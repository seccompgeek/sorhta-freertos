/* STM32MP15xx Cortex-A7 Startup Code */

#include <stdint.h>

/* Stack configuration */
#define STACK_SIZE 0x2000    /* 8KB of stack space */

__attribute__ ((section(".stack")))
static uint8_t stack[STACK_SIZE];

/* Forward declarations */
extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Exception handlers */
void NMI_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler      (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler     (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler            (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler       (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler         (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler        (void) __attribute__ ((weak, alias("Default_Handler")));

/* Vector table */
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void *)&stack[STACK_SIZE],  /* Initial stack pointer */
    Reset_Handler,               /* Reset handler */
    NMI_Handler,                 /* NMI handler */
    HardFault_Handler,           /* Hard fault handler */
    MemManage_Handler,           /* MPU fault handler */
    BusFault_Handler,            /* Bus fault handler */
    UsageFault_Handler,          /* Usage fault handler */
    0,                           /* Reserved */
    0,                           /* Reserved */
    0,                           /* Reserved */
    0,                           /* Reserved */
    SVC_Handler,                 /* SVCall handler */
    DebugMon_Handler,            /* Debug monitor handler */
    0,                           /* Reserved */
    PendSV_Handler,              /* PendSV handler */
    SysTick_Handler,             /* SysTick handler */
    
    /* Add STM32MP15 specific interrupt handlers here */
};

/* Reset handler */
void Reset_Handler(void) {
    /* Initialize data and bss sections */
    extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
    
    /* Copy the data segment initializers */
    uint32_t *src = &_sidata;
    uint32_t *dest = &_sdata;
    
    while (dest < &_edata) {
        *dest++ = *src++;
    }
    
    /* Zero fill the bss segment */
    dest = &_sbss;
    while (dest < &_ebss) {
        *dest++ = 0;
    }
    
    /* Call system initialization function */
    extern void SystemInit(void);
    SystemInit();
    
    /* Call the application's entry point */
    main();
    
    /* Infinite loop */
    while (1);
}

/* Default handler for unhandled interrupts */
void Default_Handler(void) {
    while (1);
}