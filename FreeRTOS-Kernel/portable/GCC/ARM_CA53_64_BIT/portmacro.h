#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Type definitions. */
#define portCHAR        char
#define portFLOAT       float
#define portDOUBLE      double
#define portLONG        long
#define portSHORT       short
#define portSTACK_TYPE  uint64_t
#define portBASE_TYPE   int64_t

typedef portSTACK_TYPE StackType_t;
typedef int64_t BaseType_t;
typedef uint64_t UBaseType_t;

#if (configUSE_16_BIT_TICKS == 1)
    typedef uint16_t TickType_t;
    #define portMAX_DELAY (TickType_t)0xffff
#else
    typedef uint32_t TickType_t;
    #define portMAX_DELAY (TickType_t)0xffffffffUL
#endif

/* Architecture specifics. */
#define portSTACK_GROWTH            (-1)
#define portTICK_PERIOD_MS          ((TickType_t)1000 / configTICK_RATE_HZ)
#define portBYTE_ALIGNMENT          8
#define portPOINTER_SIZE_TYPE       uint64_t

/* Task function macros. */
#define portTASK_FUNCTION_PROTO(vFunction, pvParameters) void vFunction(void *pvParameters)
#define portTASK_FUNCTION(vFunction, pvParameters) void vFunction(void *pvParameters)

/* Task utilities. */
#define portYIELD()                 __asm__ __volatile__ ("svc 0" ::: "memory")
#define portEND_SWITCHING_ISR(x)    do { if(x) portYIELD(); } while(0)
#define portYIELD_FROM_ISR(x)       portEND_SWITCHING_ISR(x)

/* Port optimized task selection */
#if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

    /* Check the port selection is valid */
    #if( configMAX_PRIORITIES > 32 )
        #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 distinct priorities.
    #endif

    /* Store/clear the ready priorities in a bit map. */
    #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1ULL << ( uxPriority ) )
    #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1ULL << ( uxPriority ) )
    
    /* For ARMv8, we can use the CLZ (Count Leading Zeros) instruction */
    #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) \
        uxTopPriority = ( 31ULL - __builtin_clzll( uxReadyPriorities ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/* Forward declaration of functions needed by the port */
extern void vPortEnterCritical(void);
extern void vPortExitCritical(void);
extern void vPortYield(void);
extern void vPortRestoreTaskContext(void);

/* Critical section management. */
extern uint64_t ulPortInterruptNesting;

/* Critical section management macros */
#define portDISABLE_INTERRUPTS()    __asm__ __volatile__ ("msr daifset, #2" ::: "memory")
#define portENABLE_INTERRUPTS()     __asm__ __volatile__ ("msr daifclr, #2" ::: "memory")
#define portENTER_CRITICAL()        vPortEnterCritical()
#define portEXIT_CRITICAL()         vPortExitCritical()

#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */