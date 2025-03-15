/* 
 * linker_symbols.c - Provides definitions for symbols required by linker
 */

 #include <sys/stat.h>
 #include <stdint.h>
 #include "FreeRTOS.h"
 #include "task.h"
 
 /* FreeRTOS application hook implementations */
 void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
 {
     /* Called when a stack overflow is detected. */
     (void)xTask;
     (void)pcTaskName;
     for(;;);
 }
 
 void vApplicationMallocFailedHook(void)
 {
     /* Called if a call to pvPortMalloc() fails because there is insufficient
     free memory available. */
     for(;;);
 }
 
 /* Static memory for FreeRTOS tasks */
 #if (configSUPPORT_STATIC_ALLOCATION == 1)
 
 /* Static allocation for the IDLE task */
 static StaticTask_t xIdleTaskTCB;
 static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
 
 void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    StackType_t *pulIdleTaskStackSize)
 {
     *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
     *ppxIdleTaskStackBuffer = uxIdleTaskStack;
     *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
 }
 
 /* Static allocation for the Timer task */
 #if (configUSE_TIMERS == 1)
 static StaticTask_t xTimerTaskTCB;
 static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
 
 void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     StackType_t *pulTimerTaskStackSize)
 {
     *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
     *ppxTimerTaskStackBuffer = uxTimerTaskStack;
     *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
 }
 #endif /* configUSE_TIMERS */
 
 #endif /* configSUPPORT_STATIC_ALLOCATION */
 
 /* Heap memory for FreeRTOS */
 #if (configAPPLICATION_ALLOCATED_HEAP == 1)
 uint8_t ucHeap[configTOTAL_HEAP_SIZE];
 #endif
 
 /* FreeRTOS port specific variables from portASM.S */
 uint64_t ulPortTaskHasFPUContext = 0;
 
 /* Simplified _sbrk implementation that uses the ucHeap array for memory allocation */
 void *_sbrk(int incr) {
     /* Use our FreeRTOS heap */
     static uint8_t *heap_end = NULL;
     uint8_t *prev_heap_end;
 
     if (heap_end == NULL) {
         heap_end = ucHeap;
     }
 
     /* Make sure we don't exceed the heap size */
     if ((heap_end + incr) > (ucHeap + configTOTAL_HEAP_SIZE)) {
         return (void*)-1;
     }
 
     prev_heap_end = heap_end;
     heap_end += incr;
 
     return (void*)prev_heap_end;
 }

 // Add to linker_symbols.c
int _write(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

int _close(int file) {
    (void)file;
    return -1;
}

int _lseek(int file, int ptr, int dir) {
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

int _read(int file, char *ptr, int len) {
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

void _exit(int status) {
    (void)status;
    while(1);
}

int _fstat(int file, struct stat *st) {
    (void)file;
    (void)st;
    return 0;
}

int _isatty(int file) {
    (void)file;
    return 1;
}

int _kill(int pid, int sig) {
    (void)pid;
    (void)sig;
    return -1;
}

int _getpid(void) {
    return 1;
}