#include "FreeRTOS.h"
#include "task.h"

/* Global variables required by the port */
uint64_t ulPortInterruptNesting = 0;
// uint64_t ulPortTaskHasFPUContext = pdFALSE;

/* Function prototypes - implemented in portASM.S */
extern void vPortRestoreTaskContext(void);

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack,
                                   TaskFunction_t pxCode, void *pvParameters) {
  /* Setup the initial stack frame.
   * For AArch64, we need a different stack frame structure */
  pxTopOfStack--;
  *pxTopOfStack = 0x0101010101010101ULL; /* ELR_EL1 - Entry point */
  pxTopOfStack--;
  *pxTopOfStack = 0x0202020202020202ULL; /* SPSR_EL1 - Status Register */

  /* General-purpose registers */
  for (int i = 0; i < 29; i++) {
    pxTopOfStack--;
    *pxTopOfStack = 0ULL; /* X0-X28 */
  }

  /* Setup the parameters for the task */
  *(pxTopOfStack - 1) = (StackType_t)pvParameters; /* X0 */

  /* Setup the entry point */
  *(pxTopOfStack - 30) = (StackType_t)pxCode; /* ELR_EL1 */

  /* Return the new stack pointer */
  return pxTopOfStack - 30;
}

BaseType_t xPortStartScheduler(void) {
  /* Setup the timer to generate the tick. */
  vPortSetupTimerInterrupt();

  /* Start the first task. This will not return. */
  vPortRestoreTaskContext();

  /* Should not get here! */
  return 0;
}

void vPortEnterCritical(void) {
  portDISABLE_INTERRUPTS();
  ulPortInterruptNesting++;
}

void vPortExitCritical(void) {
  configASSERT(ulPortInterruptNesting);

  ulPortInterruptNesting--;
  if (ulPortInterruptNesting == 0) {
    portENABLE_INTERRUPTS();
  }
}

void vPortEndScheduler(void) {
  /* Not implemented */
  configASSERT(0);
}

void vPortSetupTimerInterrupt(void) {
  uint64_t cntfrq_el0;
  uint64_t cntp_cval_el0;

  /* Get counter frequency */
  __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r"(cntfrq_el0));

  /* Calculate timer value */
  uint64_t ticks = cntfrq_el0 / configTICK_RATE_HZ;

  /* Set compare value */
  __asm__ __volatile__("mrs %0, cntp_cval_el0" : "=r"(cntp_cval_el0));
  cntp_cval_el0 += ticks;
  __asm__ __volatile__("msr cntp_cval_el0, %0" : : "r"(cntp_cval_el0));

  /* Enable timer */
  uint64_t cntp_ctl_el0;
  __asm__ __volatile__("mrs %0, cntp_ctl_el0" : "=r"(cntp_ctl_el0));
  cntp_ctl_el0 |= 1; /* Enable bit */
  __asm__ __volatile__("msr cntp_ctl_el0, %0" : : "r"(cntp_ctl_el0));
}
