/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Adapted for STM32MP15 Cortex-A7
 */

	.text
	.arm

	.set SYS_MODE,	0x1f
	.set SVC_MODE,	0x13
	.set IRQ_MODE,	0x12

	/* Variables and functions. */
	.extern pxCurrentTCB
	.extern vTaskSwitchContext
	.extern ulPortInterruptNesting
	.extern ulPortTaskHasFPUContext
	.extern ulPortYieldRequired
	.extern ulICCIAR
	.extern ulICCEOIR

	.global vPortRestoreTaskContext
	.global vPortYield
	.global FreeRTOS_IRQ_Handler
	.global FreeRTOS_SWI_Handler
	.global ulPortInterruptNesting
	.global ulPortYieldRequired
	.global ulICCIAR
	.global ulICCEOIR

/*-----------------------------------------------------------*/

.align 4
ulICCIAR:
	.word	0
ulICCEOIR:
	.word	0
ulPortInterruptNesting:
	.word	0
ulPortYieldRequired:
	.word	0

/*-----------------------------------------------------------*/

vPortYield:
	/* Set a PendSV interrupt request to perform the context switch. */
	svc 0
	bx lr

/*-----------------------------------------------------------*/

FreeRTOS_SWI_Handler:
	/* Save the context of the current task and select a new task to run. */
	/* Note: PRESERVE8 removed as it's not supported by GCC assembler */

	/* Push r0 as we are going to use it. */
	stmdb	sp!, {r0}

	/* Supervisor mode. */
	mrs	r0, spsr
	stmdb	sp!, {r0}

	/* Push all GP registers. */
	stmdb	sp!, {r0-r12}

	/* Save the SPSR, PC, and LR. */
	ldmia	sp!, {r0}				/* Get the SPSR from the stack. */
	stmdb	sp!, {r0, lr}			/* Store SPSR and LR on the stack. */

	/* Save the critical nesting count. */
	ldr	r3, =ulPortInterruptNesting
	ldr	r0, [r3]
	stmdb	sp!, {r0}

	/* Save ulPortTaskHasFPUContext. */
	ldr	r3, =ulPortTaskHasFPUContext
	ldr	r0, [r3]
	stmdb	sp!, {r0}

	/* Save the SP in the TCB. */
	ldr	r0, =pxCurrentTCB
	ldr	r1, [r0]
	str	sp, [r1]

	/* Call the scheduler to select a new task to run. */
	cpsid	i 
	bl	vTaskSwitchContext

	/* Reset the yield demand. */
	mov	r0, #0
	ldr	r1, =ulPortYieldRequired
	str	r0, [r1]

	/* Context switch will occur after ISR exit. */
	cpsie i
	ldmfd sp!, {pc}^

/*-----------------------------------------------------------*/

FreeRTOS_IRQ_Handler:
	/* Save the context of the current task and select a new task to run. */
	/* Note: PRESERVE8 removed as it's not supported by GCC assembler */

	sub	lr, lr, #4

	/* Push the return address and SPSR. */
	stmdb	sp!, {lr}
	mrs	r14, SPSR
	stmdb	sp!, {r14}

	/* Change to supervisor mode. */
	cps	#SVC_MODE

	/* Push used registers. */
	stmdb	sp!, {r0-r3, r12, r14}

	/* Increment nesting count. The first interrupt will change the FPU context
	indicator to 0. */
	ldr	r2, =ulPortInterruptNesting
	ldr	r3, [r2]
	add	r3, r3, #1
	str	r3, [r2]

	/* Read the ICCIAR from the GIC CPU interface. */
	ldr	r1, =portICCIAR_INTERRUPT_ACKNOWLEDGE
	ldr	r0, [r1]
	
	/* Store the interrupt ID. */
	ldr r2, =ulICCIAR
	str r0, [r2]

	/* Ensure bit 2 of the GIC_CPUIF_PRIMASK is clear.  r3 contains
	ulPortInterruptNesting so it can be used as a scratch register. */
	ldr r3, =portICCPMR_PRIORITY_MASK_REGISTER
	ldr r1, [r3]
	orr r1, r1, #0x80
	str r1, [r3]
	dsb

	/* Call the interrupt handler. */
	push {r0-r3, lr}
	ldr r1, = vApplicationIRQHandler
	blx r1
	pop {r0-r3, lr}
	
	/* End of interrupt procedure. */
	dsb
	
	/* Write value read from the ICCAIR to the ICCEOIR. */
	ldr r2, =ulICCIAR
	ldr r0, [r2]
	ldr r2, =portICCEOIR_END_OF_INTERRUPT_REGISTER
	str r0, [r2]
	
	/* Restore the old nesting count. */
	ldr	r2, =ulPortInterruptNesting
	ldr	r3, [r2]
	sub	r3, r3, #1
	str	r3, [r2]

	/* A context switch is never performed if the nesting count is not 0. */
	cmp	r3, #0
	bne	exit_without_switch

	/* Did the interrupt request a context switch? If so, ulPortYieldRequired
	will have been set to a non-zero value. */
	ldr	r1, =ulPortYieldRequired
	ldr	r0, [r1]
	cmp	r0, #0
	beq	exit_without_switch

	/* Restore saved registers. */
	ldmia	sp!, {r0-r3, r12, r14}
	mov	r0, #SYS_MODE
	msr	spsr_cxsf, r0
	clrex
	add	sp, sp, #8 /* Skip past the saved pc and SPSR. */
	svc 0
	
	/* Switch to the new task. */
	vPortRestoreTaskContext:
		/* Restore the LR. */
		ldr	r0, =pxCurrentTCB
		ldr	r1, [r0]
		ldr	sp, [r1]

		/* The critical nesting depth is the first item on the stack. */
		ldmfd	sp!, {r0}

		/* Save the task's floating point context indicator. */
		ldr	r2, =ulPortTaskHasFPUContext
		str	r0, [r2]

		/* The critical nesting depth is the second item on the stack. */
		ldmfd	sp!, {r0}
		ldr	r2, =ulPortInterruptNesting
		str	r0, [r2]

		/* Restore the SPSR and PC. */
		ldmfd	sp!, {r0, r1}
		msr	spsr_cxsf, r0
		mov	lr, r1

		/* Restore all system mode registers other than the SP (which is already
		being used). */
		ldmfd	sp, {r0-r12}
		add	sp, sp, #15*4
		movs	pc, lr

exit_without_switch:
	/* Restore saved registers. */
	ldmia	sp!, {r0-r3, r12, r14}
	
	/* Leave the interrupt and return to the new task. */
	msr	SPSR_cxsf, r14
	ldmia	sp!, {lr}
	movs	pc, lr

.end