#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* S32G3 GPIO definitions - update based on your board's LED connections */
#define GPIO_BASE          0x40520000UL
#define GPIO_PDOR          0x00        /* Port Data Output Register */
#define GPIO_PSOR          0x04        /* Port Set Output Register */
#define GPIO_PCOR          0x08        /* Port Clear Output Register */
#define GPIO_PTOR          0x0C        /* Port Toggle Output Register */
#define GPIO_PDDR          0x14        /* Port Data Direction Register */

/* LED pin - update based on your board */
#define LED_PIN            5

/* Helper functions for GPIO access */
static inline void write_reg(uint64_t addr, uint32_t val)
{
    *((volatile uint32_t *)addr) = val;
}

static inline uint32_t read_reg(uint64_t addr)
{
    return *((volatile uint32_t *)addr);
}

/* Function prototypes */
static void prvSetupHardware(void);
static void prvLEDTask(void *pvParameters);
static void prvMonitorTask(void *pvParameters);

/* Global variables */
static SemaphoreHandle_t xSemaphore = NULL;

/* Main application */
int main(void)
{
    /* Configure the hardware */
    prvSetupHardware();
    
    /* Create a binary semaphore for task synchronization */
    xSemaphore = xSemaphoreCreateBinary();
    
    if(xSemaphore != NULL)
    {
        /* Give the semaphore so the LED task can take it first time */
        xSemaphoreGive(xSemaphore);
        
        /* Create the LED task */
        xTaskCreate(prvLEDTask, 
                    "LED",
                    configMINIMAL_STACK_SIZE * 2,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    NULL);
        
        /* Create the monitor task */
        xTaskCreate(prvMonitorTask,
                    "Monitor",
                    configMINIMAL_STACK_SIZE * 2,
                    NULL,
                    tskIDLE_PRIORITY + 2,
                    NULL);
        
        /* Start the scheduler */
        vTaskStartScheduler();
    }
    
    /* Should never reach here */
    for(;;);
    
    return 0;
}

/* LED task - flashes the LED */
static void prvLEDTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */
    
    const TickType_t xDelay = pdMS_TO_TICKS(500);
    
    for(;;)
    {
        /* Wait for the semaphore */
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            /* Toggle LED */
            write_reg(GPIO_BASE + GPIO_PTOR, (1 << LED_PIN));
            
            vTaskDelay(xDelay);
            
            /* Give the semaphore back */
            xSemaphoreGive(xSemaphore);
            
            /* Small delay before trying to take the semaphore again */
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/* Monitor task - monitors system stats */
static void prvMonitorTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */
    
    TickType_t xLastWakeTime;
    const TickType_t xDelay = pdMS_TO_TICKS(5000); /* 5 seconds */
    
    /* Initialize the xLastWakeTime variable with the current time */
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        /* Simply wait for the periodic execution */
        vTaskDelayUntil(&xLastWakeTime, xDelay);
        
        /* Take the semaphore */
        if(xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            /* Display some system statistics - simplified for S32G3 */
            #ifdef DEBUG_STACK_USAGE
            UBaseType_t uxHighWaterMark;
            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            #endif
            
            /* Flash the LED rapidly to indicate the monitor task is active */
            for (int i = 0; i < 5; i++)
            {
                write_reg(GPIO_BASE + GPIO_PSOR, (1 << LED_PIN));
                vTaskDelay(pdMS_TO_TICKS(100));
                write_reg(GPIO_BASE + GPIO_PCOR, (1 << LED_PIN));
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            /* Give the semaphore back */
            xSemaphoreGive(xSemaphore);
        }
    }
}

/* Hardware setup function */
static void prvSetupHardware(void)
{
    /* Note: Most hardware setup is done in SystemInit() */
    
    /* Configure LED pin as output */
    write_reg(GPIO_BASE + GPIO_PDDR, read_reg(GPIO_BASE + GPIO_PDDR) | (1 << LED_PIN));
    
    /* Turn off LED initially */
    write_reg(GPIO_BASE + GPIO_PCOR, (1 << LED_PIN));
}