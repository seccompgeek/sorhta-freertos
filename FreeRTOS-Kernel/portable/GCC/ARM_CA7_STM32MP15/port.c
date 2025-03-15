/* port.c - Cortex-A7 port for STM32MP15 */

#include "FreeRTOS.h"
#include "task.h"

/* Constants required to setup the task context */
#define portINITIAL_SPSR                                                       \
  ((StackType_t)0x1f) /* System mode, ARM mode, IRQ enabled FIQ enabled. */
#define portTHUMB_MODE_BIT ((StackType_t)0x20)
#define portINSTRUCTION_SIZE ((StackType_t)4)
#define portNO_CRITICAL_SECTION_NESTING ((StackType_t)0)

/* Interrupt controller access addresses for STM32MP15 */
#define portICCPMR_PRIORITY_MASK_REGISTER 0xA0021004
#define portICCIAR_INTERRUPT_ACKNOWLEDGE 0xA002000C
#define portICCEOIR_END_OF_INTERRUPT_REGISTER 0xA0020010
#define portICCBPR_BINARY_POINT_REGISTER 0xA0021008
#define portICCRPR_RUNNING_PRIORITY_REGISTER 0xA0021014

/* GIC Distributor interface registers for STM32MP15 */
#define portGIC_DISTRIBUTOR_BASE 0xA0011000UL
#define portGIC_SET_ENABLE_0 0x100
#define portGIC_SET_ENABLE_1 0x104
#define portGIC_SET_ENABLE_2 0x108

/*-----------------------------------------------------------*/

/*
 * Starts the first task by restoring its context and jumping to the task.
 */
void vPortRestoreTaskContext(void);

/*
 * We need to define our own port yield function for Cortex-A7
 */
void vPortYield(void);

/*-----------------------------------------------------------*/

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack,
                                   TaskFunction_t pxCode, void *pvParameters) {
  /* Setup the initial stack of the task.  The stack is set exactly as
  expected by the portRESTORE_CONTEXT() macro. */

  /* First all the general purpose registers. */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R0 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R1 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R2 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R3 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R4 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R5 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R6 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R7 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R8 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R9 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R10 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R11 */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0x00000000; /* R12 */

  /* The task will start with a critical nesting count of 0 as interrupts are
  enabled. */
  pxTopOfStack--;
  *pxTopOfStack = portNO_CRITICAL_SECTION_NESTING;

  /* The task will start without a floating point context. The task may create
  a floating point context, in which case a context save area will be created
  and this variable will be set to a non-zero value. */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0;

  /* Setup the SPSR value for running in Supervisor mode. */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)portINITIAL_SPSR;

  if (((uint32_t)pxCode & portTHUMB_MODE_BIT) != 0x00UL) {
    /* The task will start in THUMB mode. */
    *pxTopOfStack |= portTHUMB_MODE_BIT;
  }

  /* Setup the return address. */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)pxCode; /* PC */

  /* r0 contains the parameter. */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)pvParameters; /* R0 */

  /* SVC Link Register. */
  pxTopOfStack--;
  *pxTopOfStack = (StackType_t)0; /* LR */

  /* Save the SVC stack pointer into the TCB. */
  return pxTopOfStack;
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler(void) {
  // uint32_t ulValue;

  /* Setup the timer to generate the tick. */
  configSETUP_TICK_INTERRUPT();

  /* Start the first task. */
  vPortRestoreTaskContext();

  /* Should not get here as the tasks are now running! */
  return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler(void) {
  /* Not implemented. */
  configASSERT(0);
}
/*-----------------------------------------------------------*/

void vPortEnterCritical(void) {
  portDISABLE_INTERRUPTS();
  ulPortInterruptNesting++;
}
/*-----------------------------------------------------------*/

void vPortExitCritical(void) {
  configASSERT(ulPortInterruptNesting);

  ulPortInterruptNesting--;
  if (ulPortInterruptNesting == 0) {
    portENABLE_INTERRUPTS();
  }
}
