#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* S32G3 memory-mapped registers */
#define GIC_DISTRIBUTOR_BASE    0x50800000UL
#define GIC_REDISTRIBUTOR_BASE  0x50880000UL
#define GIC_CPU_INTERFACE_BASE  0x50900000UL

/* LED GPIO registers - replace with actual S32G3 values */
#define GPIO_BASE               0x40520000UL
#define GPIO_PDOR               0x00        /* Port Data Output Register */
#define GPIO_PSOR               0x04        /* Port Set Output Register */
#define GPIO_PCOR               0x08        /* Port Clear Output Register */
#define GPIO_PTOR               0x0C        /* Port Toggle Output Register */
#define GPIO_PDDR               0x14        /* Port Data Direction Register */

/* Helper functions */
static inline void write_reg(uint64_t addr, uint32_t val)
{
    *((volatile uint32_t *)addr) = val;
}

static inline uint32_t read_reg(uint64_t addr)
{
    return *((volatile uint32_t *)addr);
}

/* Configure LED GPIO */
static void configure_led_gpio(void)
{
    /* Configure GPIO for LED output - actual pins will vary by board */
    write_reg(GPIO_BASE + GPIO_PDDR, read_reg(GPIO_BASE + GPIO_PDDR) | (1 << 5)); /* Set pin 5 as output */
}

/* Configure GICv3 */
static void configure_gic(void)
{
    /* Initialize GIC Distributor */
    write_reg(GIC_DISTRIBUTOR_BASE + 0x0, 0);   /* Disable distributor */
    
    /* TODO: Configure interrupt priorities, targets, and enable specific interrupts */
    
    /* Enable distributor */
    write_reg(GIC_DISTRIBUTOR_BASE + 0x0, 1);
    
    /* Initialize GIC CPU interface */
    /* For GICv3, we need to configure the redistributor */
    /* TODO: Configure redistributor */
    
    /* Enable system timer interrupt */
    /* TODO: Configure system timer for interrupt generation */
}

/* Configure MMU */
static void configure_mmu(void)
{
    /* Basic MMU setup for direct mapping (virtual = physical) */
    uint64_t tcr_el1 = 0;
    uint64_t sctlr_el1 = 0;
    
    /* Configure Translation Control Register */
    tcr_el1 |= (1ULL << 20);   /* TBI0: Top Byte Ignored */
    tcr_el1 |= (1ULL << 8);    /* IRGN0: Inner Write-Back Read-Allocate Write-Allocate Cacheable */
    tcr_el1 |= (1ULL << 10);   /* ORGN0: Outer Write-Back Read-Allocate Write-Allocate Cacheable */
    tcr_el1 |= (1ULL << 12);   /* SH0: Inner Shareable */
    tcr_el1 |= 25ULL;          /* T0SZ: 39-bit addressing */
    
    /* Set Translation Control Register */
    __asm__ __volatile__("msr tcr_el1, %0" : : "r" (tcr_el1));
    
    /* Set Memory Attribute Indirection Register */
    uint64_t mair_el1 = 0;
    mair_el1 |= 0xFF << 0;     /* Attr0: Normal, Write-Back */
    mair_el1 |= 0x04 << 8;     /* Attr1: Device-nGnRE */
    __asm__ __volatile__("msr mair_el1, %0" : : "r" (mair_el1));
    
    /* Setup identity mapping in translation tables */
    /* TODO: Create and configure translation tables */
    
    /* Enable MMU */
    __asm__ __volatile__("mrs %0, sctlr_el1" : "=r" (sctlr_el1));
    sctlr_el1 |= (1 << 0);     /* M: Enable MMU */
    sctlr_el1 |= (1 << 2);     /* C: Enable data cache */
    sctlr_el1 |= (1 << 12);    /* I: Enable instruction cache */
    __asm__ __volatile__("msr sctlr_el1, %0" : : "r" (sctlr_el1));
    
    /* Instruction synchronization barrier */
    __asm__ __volatile__("isb");
}

/* Main system initialization function */
void SystemInit(void)
{
    /* Basic hardware setup */
    
    /* Configure MMU */
    configure_mmu();
    
    /* Configure LED GPIO */
    configure_led_gpio();
    
    /* Configure GIC */
    configure_gic();
}

/* Handler for Generic Timer interrupt */
void FreeRTOS_Tick_Handler(void)
{
    /* Increment the RTOS tick */
    BaseType_t xSwitchRequired = xTaskIncrementTick();
    
    /* Setup next tick */
    uint64_t cntfrq_el0;
    uint64_t cntp_cval_el0;
    
    /* Get counter frequency */
    __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r" (cntfrq_el0));
    
    /* Calculate next tick time */
    uint64_t ticks = cntfrq_el0 / configTICK_RATE_HZ;
    
    /* Set new compare value */
    __asm__ __volatile__("mrs %0, cntp_cval_el0" : "=r" (cntp_cval_el0));
    cntp_cval_el0 += ticks;
    __asm__ __volatile__("msr cntp_cval_el0, %0" : : "r" (cntp_cval_el0));
    
    /* Check if context switch required */
    if (xSwitchRequired != pdFALSE)
    {
        extern uint64_t ulPortYieldRequired;
        ulPortYieldRequired = 1;
    }
}