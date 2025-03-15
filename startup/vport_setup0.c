/*
 * Implementation of vPortSetupTimerInterrupt for STM32MP15
 */

 #include "FreeRTOS.h"
 #include "task.h"
 
 /* This is called from xPortStartScheduler() */
 void vPortSetupTimerInterrupt(void)
 {
     /* The SystemFrequency variable is set by SystemInit(). */
     /* Configure Generic Timer to generate tick interrupt */
     
     /* This function is defined in the SystemInit.c, which properly configures
        the ARM Generic Timer for generating ticks */
     extern void configure_systick(void);
     configure_systick();
 }