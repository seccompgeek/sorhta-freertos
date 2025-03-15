/*
 * FreeRTOS Kernel V10.5.1
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Custom heap setup for STM32MP15
 * 
 * This file ensures that we have the heap_4.c implementation loaded and properly configured
 */

 #include "FreeRTOS.h"
 #include "task.h"
 
 /* Define the heap regions for heap_4.c */
 #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
 
 /* Heap_4.c needs the following definitions from FreeRTOSConfig.h:
  * - configTOTAL_HEAP_SIZE: Total size of the heap in bytes
  * - configAPPLICATION_ALLOCATED_HEAP: Set to 1 if the application provides the heap
  */
 
 /* The heap is defined in the linker script but we need a pointer to it for FreeRTOS */
 #if (configAPPLICATION_ALLOCATED_HEAP == 1)
 /* If the application provides the heap, we need to define it here */
 #include <stddef.h>
 
 /* The linker script defines these symbols */
 extern uint8_t __heap_start__[];
 extern uint8_t __heap_end__[];
 
 /* Calculate heap size from linker symbols */
 const size_t xHeapSize = (size_t)(__heap_end__ - __heap_start__);
 
 /* Provide the heap to FreeRTOS */
 uint8_t *ucHeap = __heap_start__;
 #endif /* configAPPLICATION_ALLOCATED_HEAP */
 
 #endif /* configSUPPORT_DYNAMIC_ALLOCATION */
 
 /* Provide an implementation for vApplicationMallocFailedHook */
 void vApplicationMallocFailedHook(void)
 {
     /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available. */
     for(;;);
 }