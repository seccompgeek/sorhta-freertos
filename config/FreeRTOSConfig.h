#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Basic configuration */
#define configUSE_PREEMPTION                1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE             0
#define configCPU_CLOCK_HZ                  800000000
#define configTICK_RATE_HZ                  1000
#define configMAX_PRIORITIES                32
#define configMINIMAL_STACK_SIZE            256
#define configTOTAL_HEAP_SIZE               (1024 * 1024)
#define configMAX_TASK_NAME_LEN             16
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             1
#define configUSE_TASK_NOTIFICATIONS        1
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         1
#define configUSE_COUNTING_SEMAPHORES       1
#define configQUEUE_REGISTRY_SIZE           8
#define configUSE_QUEUE_SETS                1
#define configUSE_TIME_SLICING              1
#define configUSE_NEWLIB_REENTRANT          0
#define configENABLE_BACKWARD_COMPATIBILITY 0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* Memory allocation settings */
#define configSUPPORT_STATIC_ALLOCATION     1
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configAPPLICATION_ALLOCATED_HEAP    1

/* Hook function settings */
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCHECK_FOR_STACK_OVERFLOW      2
#define configUSE_MALLOC_FAILED_HOOK        1

/* Run time stats gathering definitions */
#define configGENERATE_RUN_TIME_STATS       0
#define configUSE_TRACE_FACILITY            1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Co-routine definitions */
#define configUSE_CO_ROUTINES               0
#define configMAX_CO_ROUTINE_PRIORITIES     2

/* Software timer definitions */
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH            10
#define configTIMER_TASK_STACK_DEPTH        (configMINIMAL_STACK_SIZE * 2)

/* Cortex-A specific settings */
#define configMAX_API_CALL_INTERRUPT_PRIORITY configMAX_PRIORITIES

/* Define to trap errors during development */
#define configASSERT(x) if((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}

/* Set the following definitions to 1 to include the API function, or zero to exclude */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_xResumeFromISR              1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTaskGetCurrentTaskHandle   1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle      1
#define INCLUDE_eTaskGetState               1
#define INCLUDE_xEventGroupSetBitFromISR    1
#define INCLUDE_xTimerPendFunctionCall      1
#define INCLUDE_xTaskAbortDelay             1
#define INCLUDE_xTaskGetHandle              1
#define INCLUDE_xTaskResumeFromISR          1

/* S32G3 specific settings */
#define configUSE_DAEMON_TASK_STARTUP_HOOK  0
#define configENABLE_FPU                    1
#define configENABLE_TRUSTZONE              0

/* GICv3 interrupt controller settings */
#define configINTERRUPT_CONTROLLER_BASE_ADDRESS         0x50800000UL
#define configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET 0x100000UL
#define configGIC_DISTRIBUTOR_PRIORITY_BITS             3
#define configUNIQUE_INTERRUPT_PRIORITIES               32

/* This prototype is required for setup of interrupts */
void vPortSetupTimerInterrupt(void);
#define configSETUP_TICK_INTERRUPT() vPortSetupTimerInterrupt()

/* Ensure bits are cleared for ARM_A53 port */
#define configCLEAR_TICK_INTERRUPT() /* Done automatically in hardware */

#endif /* FREERTOS_CONFIG_H */