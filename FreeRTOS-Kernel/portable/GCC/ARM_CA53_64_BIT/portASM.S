.section .text
.global vPortRestoreTaskContext
.global FreeRTOS_IRQ_Handler
.global vPortYield

// External references
.extern pxCurrentTCB
.extern ulPortInterruptNesting
.extern vTaskSwitchContext
.extern ulPortYieldRequired
.extern FreeRTOS_Tick_Handler

// Macro to save context
.macro SAVE_CONTEXT
    // Make space on the stack for context
    sub sp, sp, #272
    
    // Save general-purpose registers
    stp x0, x1, [sp, #0]
    stp x2, x3, [sp, #16]
    stp x4, x5, [sp, #32]
    stp x6, x7, [sp, #48]
    stp x8, x9, [sp, #64]
    stp x10, x11, [sp, #80]
    stp x12, x13, [sp, #96]
    stp x14, x15, [sp, #112]
    stp x16, x17, [sp, #128]
    stp x18, x19, [sp, #144]
    stp x20, x21, [sp, #160]
    stp x22, x23, [sp, #176]
    stp x24, x25, [sp, #192]
    stp x26, x27, [sp, #208]
    stp x28, x29, [sp, #224]
    
    // Save LR and ELR_EL1
    mrs x0, elr_el1
    mrs x1, spsr_el1
    stp x30, x0, [sp, #240]
    str x1, [sp, #256]
.endm

// Macro to restore context
.macro RESTORE_CONTEXT
    // Restore general-purpose registers
    ldp x0, x1, [sp, #0]
    ldp x2, x3, [sp, #16]
    ldp x4, x5, [sp, #32]
    ldp x6, x7, [sp, #48]
    ldp x8, x9, [sp, #64]
    ldp x10, x11, [sp, #80]
    ldp x12, x13, [sp, #96]
    ldp x14, x15, [sp, #112]
    ldp x16, x17, [sp, #128]
    ldp x18, x19, [sp, #144]
    ldp x20, x21, [sp, #160]
    ldp x22, x23, [sp, #176]
    ldp x24, x25, [sp, #192]
    ldp x26, x27, [sp, #208]
    ldp x28, x29, [sp, #224]
    
    // Restore LR and ELR_EL1
    ldp x30, x0, [sp, #240]
    ldr x1, [sp, #256]
    msr elr_el1, x0
    msr spsr_el1, x1
    
    // Restore stack pointer
    add sp, sp, #272
.endm

// SVC handler for yield
vPortYield:
    // Generate an SVC to cause a context switch
    svc #0
    ret

// Function to restore task context
vPortRestoreTaskContext:
    // Load the address of pxCurrentTCB
    adrp x0, pxCurrentTCB
    add x0, x0, :lo12:pxCurrentTCB
    ldr x1, [x0]
    
    // Get stack pointer from TCB
    ldr x2, [x1]
    mov sp, x2
    
    // Restore context and return
    RESTORE_CONTEXT
    eret

// SVC Handler
.global FreeRTOS_SVC_Handler
FreeRTOS_SVC_Handler:
    // Save context
    SAVE_CONTEXT
    
    // Save the current stack pointer to the TCB
    adrp x0, pxCurrentTCB
    add x0, x0, :lo12:pxCurrentTCB
    ldr x1, [x0]
    mov x2, sp
    str x2, [x1]
    
    // Call the scheduler to select a new task
    bl vTaskSwitchContext
    
    // Get the new task stack pointer
    adrp x0, pxCurrentTCB
    add x0, x0, :lo12:pxCurrentTCB
    ldr x1, [x0]
    ldr x2, [x1]
    mov sp, x2
    
    // Restore context and return
    RESTORE_CONTEXT
    eret

// IRQ Handler
FreeRTOS_IRQ_Handler:
    // Save context
    SAVE_CONTEXT
    
    // Increment nesting count
    adrp x0, ulPortInterruptNesting
    add x0, x0, :lo12:ulPortInterruptNesting
    ldr x1, [x0]
    add x1, x1, #1
    str x1, [x0]
    
    // Save the current stack pointer to the TCB
    adrp x0, pxCurrentTCB
    add x0, x0, :lo12:pxCurrentTCB
    ldr x1, [x0]
    mov x2, sp
    str x2, [x1]
    
    // Call C function to handle the interrupt
    bl FreeRTOS_Tick_Handler
    
    // Decrement nesting count
    adrp x0, ulPortInterruptNesting
    add x0, x0, :lo12:ulPortInterruptNesting
    ldr x1, [x0]
    sub x1, x1, #1
    str x1, [x0]
    
    // Check if yield required
    adrp x0, ulPortYieldRequired
    add x0, x0, :lo12:ulPortYieldRequired
    ldr x1, [x0]
    cbnz x1, context_switch
    
    // No context switch needed, restore context and return
    RESTORE_CONTEXT
    eret

context_switch:
    // Clear yield required flag
    mov x2, #0
    str x2, [x0]
    
    // Switch context
    bl vTaskSwitchContext
    
    // Get new task's stack pointer
    adrp x0, pxCurrentTCB
    add x0, x0, :lo12:pxCurrentTCB
    ldr x1, [x0]
    ldr x2, [x1]
    mov sp, x2
    
    // Restore context and return
    RESTORE_CONTEXT
    eret