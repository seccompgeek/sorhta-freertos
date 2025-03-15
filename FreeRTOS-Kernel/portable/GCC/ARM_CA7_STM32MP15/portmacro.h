/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Adapted for STM32MP15 Cortex-A7
 */

 #ifndef PORTMACRO_H
 #define PORTMACRO_H
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /*-----------------------------------------------------------
  * Port specific definitions.
  *
  * The settings in this file configure FreeRTOS correctly for the
  * given hardware and compiler.
  *
  * These settings should not be altered.
  *-----------------------------------------------------------
  */
 
 /* Type definitions. */
 #define portCHAR		char
 #define portFLOAT		float
 #define portDOUBLE		double
 #define portLONG		long
 #define portSHORT		short
 #define portSTACK_TYPE	uint32_t
 #define portBASE_TYPE	long
 
 typedef portSTACK_TYPE StackType_t;
 typedef long BaseType_t;
 typedef unsigned long UBaseType_t;
 
 #if( configUSE_16_BIT_TICKS == 1 )
     typedef uint16_t TickType_t;
     #define portMAX_DELAY ( TickType_t ) 0xffff
 #else
     typedef uint32_t TickType_t;
     #define portMAX_DELAY ( TickType_t ) 0xffffffffUL
 
     /* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
     not need to be guarded with a critical section. */
     #define portTICK_TYPE_IS_ATOMIC 1
 #endif
 /*-----------------------------------------------------------*/
 
 /* Architecture specifics. */
 #define portSTACK_GROWTH			( -1 )
 #define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
 #define portBYTE_ALIGNMENT			8
 #define portPOINTER_SIZE_TYPE		uint32_t
 /*-----------------------------------------------------------*/
 
 /* Task utilities. */
 
 /* Add port optimized task selection functions */
 #if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1
 
     /* Store/clear the ready priorities in a bit map. */
     #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
     #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )
     #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31UL - ( uint32_t ) __builtin_clz( uxReadyPriorities ) )
 
 #endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */
 
 /* Called at the end of an ISR that can cause a context switch. */
 #define portEND_SWITCHING_ISR( xSwitchRequired )\
 {												\
     extern uint32_t ulPortYieldRequired;		\
                                                 \
     if( xSwitchRequired != pdFALSE )			\
     {											\
         ulPortYieldRequired = pdTRUE;			\
     }											\
 }
 
 #define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
 #define portYIELD() vPortYield()
 
 /*-----------------------------------------------------------
  * Critical section control
  *----------------------------------------------------------*/
 
 extern void vPortEnterCritical( void );
 extern void vPortExitCritical( void );
 extern void vPortYield( void );
 extern uint32_t ulPortInterruptNesting;
 
 #define portDISABLE_INTERRUPTS()											\
     __asm volatile ( "CPSID i" ::: "memory" );								\
     __asm volatile ( "DSB" );												\
     __asm volatile ( "ISB" );
 
 #define portENABLE_INTERRUPTS()												\
     __asm volatile ( "CPSIE i" ::: "memory" );								\
     __asm volatile ( "DSB" );												\
     __asm volatile ( "ISB" );
 
 /* These macros should be called directly, but through the taskENTER_CRITICAL()
 and taskEXIT_CRITICAL() macros. */
 #define portENTER_CRITICAL() vPortEnterCritical()
 #define portEXIT_CRITICAL() vPortExitCritical()
 
 /* Task function macros as described on the FreeRTOS.org WEB site. */
 #define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
 #define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* PORTMACRO_H */