#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "print.h"

/* S32G3 specific registers */
#define MC_ME_BASE               0x40088000
#define MC_ME_CTL_KEY            0x40088000
#define MC_ME_MODE_CONF          0x40088004
#define MC_ME_MODE_UPD           0x40088008
#define MC_ME_MODE_STAT          0x4008800C
#define MC_ME_CORE_PCONF(n)      (0x40088100 + ((n) * 4))
#define MC_ME_CORE_PUPD(n)       (0x40088180 + ((n) * 4))
#define MC_ME_CORE_STAT(n)       (0x40088200 + ((n) * 4))

/* MC_ME keys for mode entry */
#define MC_ME_KEY1               0x5AF0
#define MC_ME_KEY2               0xA50F

/* GIC-500 base addresses */
#define GIC_DIST_BASE            0x50800000
/* GIC CPU registers */
#define GIC_CPU_BASE            0x50900000
#define GICC_IAR                (GIC_CPU_BASE + 0x000C)
#define GICC_EOIR               (GIC_CPU_BASE + 0x0010)

/* GICD (Distributor) registers */
#define GICD_CTLR                (GIC_DIST_BASE + 0x0000)
#define GICD_TYPER               (GIC_DIST_BASE + 0x0004)
#define GICD_ISENABLER(n)        (GIC_DIST_BASE + 0x0100 + ((n) * 4))
#define GICD_IPRIORITY(n)        (GIC_DIST_BASE + 0x0400 + ((n) * 4))
#define GICD_ITARGETSR(n)        (GIC_DIST_BASE + 0x0800 + ((n) * 4))
#define GICD_ICFGR(n)            (GIC_DIST_BASE + 0x0C00 + ((n) * 4))

/* GICC (CPU Interface) registers */
#define GICC_CTLR                (GIC_CPU_BASE + 0x0000)
#define GICC_PMR                 (GIC_CPU_BASE + 0x0004)
#define GICC_IAR                 (GIC_CPU_BASE + 0x000C)
#define GICC_EOIR                (GIC_CPU_BASE + 0x0010)

/* L2 cache controller (CCN-5xx) */
#define CCI_BASE                 0x4052000  /* Cache Coherent Interconnect base */

/* IPC (Inter-Processor Communication) registers for S32G3 */
#define IPC_BASE                 0x4006C000
#define IPC_MSG_SR(n)            (IPC_BASE + 0x00 + ((n) * 4))  /* Status Register */
#define IPC_MSG_MR(n)            (IPC_BASE + 0x40 + ((n) * 4))  /* Message Register */

/* Memory regions for FreeRTOS on Core 1 */
#define FREERTOS_CODE_BASE       0xE0000000
#define FREERTOS_CODE_SIZE       0x00100000  /* 1MB for code */
#define FREERTOS_HEAP_BASE       0xE0100000
#define FREERTOS_HEAP_SIZE       0x00F00000  /* 15MB for heap/stack */

/* Constants for core control */
#define CORE_ID_MASK             0x0F
#define CORE_RUNNING             0x01
#define CORE_READY               0x02
#define CORE_ERROR               0x80
#define CORE_FREERTOS_RUNNING    0x03

/* Page table definitions */
#define PAGE_SIZE                0x1000      /* 4KB pages */
#define NUM_PAGE_TABLE_ENTRIES   4096        /* Cover 16MB of address space */
#define NUM_L1_ENTRIES           512         /* Number of L1 entries */

/* Memory attributes for MAIR_EL1 */
#define MAIR_ATTR_DEVICE_nGnRnE  0x00        /* Device, non-Gathering, non-Reordering, no Early write acknowledgement */
#define MAIR_ATTR_NORMAL_NC      0x44        /* Normal, non-cacheable */
#define MAIR_ATTR_NORMAL_WT      0xBB        /* Normal, write-through */
#define MAIR_ATTR_NORMAL_WB      0xFF        /* Normal, write-back, read-allocate, write-allocate */

/* TCR_EL1 definitions */
#define TCR_T0SZ                 (64 - 36)   /* 36-bit address space (64GB) */
#define TCR_IRGN0_WBWA           (1 << 8)    /* Inner write-back, write-allocate */
#define TCR_ORGN0_WBWA           (1 << 10)   /* Outer write-back, write-allocate */
#define TCR_SH0_INNER            (3 << 12)   /* Inner shareable */
#define TCR_TG0_4K               (0 << 14)   /* 4KB granule size */
#define TCR_EL1_VALUE            (TCR_T0SZ | TCR_IRGN0_WBWA | TCR_ORGN0_WBWA | TCR_SH0_INNER | TCR_TG0_4K)

/* SCTLR_EL1 bits */
#define SCTLR_M                  (1 << 0)    /* MMU enable */
#define SCTLR_C                  (1 << 2)    /* Data cache enable */
#define SCTLR_I                  (1 << 12)   /* Instruction cache enable */

/* Memory regions for page tables */
#define REGION_DEVICE            0
#define REGION_NORMAL_NC         1
#define REGION_NORMAL_WT         2
#define REGION_NORMAL_WB         3

/* L1 descriptor attributes */
#define L1_TABLE                 0x3         /* Table descriptor */
#define L1_BLOCK                 0x1         /* Block descriptor */

/* L2 descriptor attributes */
#define L2_PAGE                  0x3         /* Page descriptor */
#define ATTR_DEVICE_nGnRnE       (0x0 << 2)  /* Index to MAIR_EL1 for device memory */
#define ATTR_NORMAL_WB           (0x3 << 2)  /* Index to MAIR_EL1 for normal memory */
#define ACCESS_FLAG              (1 << 10)   /* Access flag */
#define SHAREABLE                (2 << 8)    /* Inner shareable */
#define AP_RW_EL1                (0 << 6)    /* RW at EL1, no access at EL0 */
#define AP_RW_ALL                (1 << 6)    /* RW at all levels */
#define NON_GLOBAL               (0 << 11)   /* Non-global page */
#define GLOBAL                   (1 << 11)   /* Global page */

/* Multi-core boot control flags - placed in shared memory */
#define SHARED_MEM_BASE          0x80F00000  /* Based on your memory map */


/* Global variable for tracking if a context switch is needed */
uint64_t ulPortYieldRequired = 0;


/* Memory barriers */
#define DMB() __asm volatile("dmb ish" : : : "memory")
#define DSB() __asm volatile("dsb ish" : : : "memory")
#define ISB() __asm volatile("isb" : : : "memory")

/* Code region boundaries */
extern uint8_t __code_region_start[];
extern uint8_t __code_region_end[];

/* Data region boundaries */
extern uint8_t __data_region_start[];
extern uint8_t __data_region_end[];

/* Page tables region boundaries */
extern uint8_t __page_tables_start[];
extern uint8_t __page_tables_end[];

/* Heap region boundaries */
extern uint8_t __heap_region_start[];
extern uint8_t __heap_region_end[];
extern uint8_t __heap_start[];
extern uint8_t __heap_end[];

/* Stack region boundaries */
extern uint8_t __stack_region_start[];
extern uint8_t __stack_region_end[];
extern uint8_t _stack_bottom[];
extern uint8_t _stack_top[];

/* FreeRTOS heap information */
extern uint8_t __freertos_heap_start[];
extern uintptr_t __freertos_heap_size;

/* Section boundaries */
extern uint8_t _sdata[];
extern uint8_t _edata[];
extern uint8_t _sbss[];
extern uint8_t _ebss[];
extern uint8_t _etext[];
extern uint8_t _end[];

/* 
 * Define page tables using linker-provided memory region.
 * The linker places these in the .page_tables section.
 */
__attribute__((section(".page_tables"), aligned(PAGE_SIZE)))
static uint64_t l1_table[512];

__attribute__((section(".page_tables"), aligned(PAGE_SIZE)))
static uint64_t l2_table_code[512];

__attribute__((section(".page_tables"), aligned(PAGE_SIZE)))
static uint64_t l2_table_periph[512];


/* Structure for inter-core communication */
typedef struct {
    volatile uint32_t core_status[8];      /* Status of each core */
    volatile uint32_t boot_stage;          /* Current boot stage */
    volatile uint32_t handoff_complete;    /* Flag indicating when AT-F can proceed */
    volatile uint32_t jump_addr[8];        /* Jump addresses for each core */
    volatile uint32_t args[8][4];          /* Arguments for each core */
    volatile uint32_t lock;                /* Global lock for boot control structure */
} boot_control_t;

/* Place boot control structure in shared memory */
static boot_control_t *const boot_ctrl = (boot_control_t *)SHARED_MEM_BASE;

/* Forward declarations for FreeRTOS */
extern void vPortStartFirstTask(void);
extern void FreeRTOS_SetupTickInterrupt(void);

/**
 * Detect which core this code is running on
 */
static uint32_t get_current_core_id(void)
{
    uint64_t mpidr;
    __asm volatile("mrs %0, mpidr_el1" : "=r" (mpidr));
    return (uint32_t)(mpidr & CORE_ID_MASK);
}

/**
 * Simple spin lock implementation for critical sections
 */
static void spin_lock(volatile uint32_t *lock)
{
    while (1) {
        uint32_t val;
        /* Use load-exclusive to check current lock value */
        __asm volatile("ldaxr %w0, [%1]" : "=r" (val) : "r" (lock) : "memory");
        
        if (val == 0) {
            /* Try to acquire lock with store-exclusive */
            uint32_t failure;
            __asm volatile("stxr %w0, %w1, [%2]" 
                         : "=&r" (failure)
                         : "r" (1), "r" (lock)
                         : "memory");
                         
            if (failure == 0) {
                /* Lock acquired successfully */
                DMB();
                break;
            }
        }
        
        /* If lock is held, yield or wait briefly */
        __asm volatile("wfe");
    }
}

/**
 * Release spin lock
 */
static void spin_unlock(volatile uint32_t *lock)
{
    DMB();
    __asm volatile("stlr wzr, [%0]" : : "r" (lock) : "memory");
    __asm volatile("sev"); /* Signal event to wake potential waiters */
}

/**
 * Write to a memory-mapped register
 */
static inline void write_reg(uint32_t addr, uint32_t value)
{
    *((volatile uint32_t *)addr) = value;
    DSB();
}

/**
 * Read from a memory-mapped register
 */
static inline uint32_t read_reg(uint32_t addr)
{
    uint32_t value = *((volatile uint32_t *)addr);
    DSB();
    return value;
}

/**
 * Send a message to another core
 */
static void send_ipc_message(uint32_t target_core, uint32_t message)
{
    write_reg(IPC_MSG_MR(target_core), message);
    write_reg(IPC_MSG_SR(target_core), 1);
    
    /* Send event to wake up target core if it's waiting */
    __asm volatile("sev");
}

/**
 * Set core status in shared control structure
 */
static void set_core_status(uint32_t core_id, uint32_t status)
{
    spin_lock(&boot_ctrl->lock);
    boot_ctrl->core_status[core_id] = status;
    spin_unlock(&boot_ctrl->lock);
}

/**
 * Initialize page tables using linker-defined addresses.
 * Maps only the necessary memory regions.
 */
static void init_page_tables(void)
{
    /* Get addresses from linker script symbols */
    uintptr_t code_start = (uintptr_t)__code_region_start;
    uintptr_t code_end = (uintptr_t)__code_region_end;
    uintptr_t data_start = (uintptr_t)__data_region_start;
    uintptr_t data_end = (uintptr_t)__data_region_end;
    uintptr_t heap_start = (uintptr_t)__heap_start;
    uintptr_t heap_end = (uintptr_t)__heap_end;
    uintptr_t stack_start = (uintptr_t)_stack_bottom;
    uintptr_t stack_end = (uintptr_t)_stack_top;
    uintptr_t page_tables_start = (uintptr_t)__page_tables_start;
    uintptr_t page_tables_end = (uintptr_t)__page_tables_end;
    
    /* Clear all tables first */
    for (int i = 0; i < 512; i++) {
        l1_table[i] = 0;
        l2_table_code[i] = 0;
        l2_table_periph[i] = 0;
    }
    
    /* Calculate L1 indices for memory regions */
    uint32_t code_l1_idx = (code_start >> 30) & 0x1FF;   /* Bits [38:30] */
    uint32_t periph_l1_idx = (0x40000000 >> 30) & 0x1FF; /* Peripherals at 0x40000000 */
    
    /* Set up L1 entries to point to L2 tables */
    l1_table[code_l1_idx] = ((uintptr_t)&l2_table_code) | L1_TABLE;
    l1_table[periph_l1_idx] = ((uintptr_t)&l2_table_periph) | L1_TABLE;
    
    /* Map code and data regions in L2 table */
    uint32_t num_pages = 0;
    uintptr_t start_addr = 0;
    uintptr_t end_addr = 0;
    uint32_t l2_idx = 0;
    uint64_t attrs = 0;
    
    /* Map code region - Read-only executable */
    start_addr = code_start;
    end_addr = code_end;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;  /* Use bits [20:12] of addr for L2 index */
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_NORMAL_WB | ACCESS_FLAG | SHAREABLE | AP_RW_EL1 | GLOBAL;
        l2_table_code[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* Map data region - Read-write data */
    start_addr = data_start;
    end_addr = data_end;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_NORMAL_WB | ACCESS_FLAG | SHAREABLE | AP_RW_ALL | GLOBAL;
        l2_table_code[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* Map heap region - Read-write */
    start_addr = heap_start;
    end_addr = heap_end;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_NORMAL_WB | ACCESS_FLAG | SHAREABLE | AP_RW_ALL | GLOBAL;
        l2_table_code[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* Map stack region - Read-write */
    start_addr = stack_start;
    end_addr = stack_end;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_NORMAL_WB | ACCESS_FLAG | SHAREABLE | AP_RW_ALL | GLOBAL;
        l2_table_code[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* Map page tables region - Read-write by privileged only */
    start_addr = page_tables_start;
    end_addr = page_tables_end;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_NORMAL_WB | ACCESS_FLAG | SHAREABLE | AP_RW_EL1 | GLOBAL;
        l2_table_code[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* Map peripheral region - Device memory */
    /* We only map the specific peripheral ranges we need */
    
    /* GIC - Interrupt controller */
    start_addr = GIC_DIST_BASE;
    end_addr = GIC_DIST_BASE + 0x10000;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_DEVICE_nGnRnE | ACCESS_FLAG | AP_RW_EL1;
        l2_table_periph[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* MC_ME - Mode Entry module */
    start_addr = MC_ME_BASE;
    end_addr = MC_ME_BASE + 0x1000;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_DEVICE_nGnRnE | ACCESS_FLAG | AP_RW_EL1;
        l2_table_periph[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
    
    /* IPC - Inter-processor communication */
    start_addr = IPC_BASE;
    end_addr = IPC_BASE + 0x1000;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;
    
    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_DEVICE_nGnRnE | ACCESS_FLAG | AP_RW_EL1;
        l2_table_periph[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }

    /* UART*/
    start_addr = UART0_BASE;
    end_addr = UART0_BASE + 0x1000;
    num_pages = (end_addr - start_addr) / PAGE_SIZE;
    l2_idx = (start_addr >> 12) & 0x1FF;

    for (uint32_t i = 0; i < num_pages && l2_idx + i < 512; i++) {
        attrs = L2_PAGE | ATTR_DEVICE_nGnRnE | ACCESS_FLAG | AP_RW_EL1;
        l2_table_periph[l2_idx + i] = (start_addr + (i * PAGE_SIZE)) | attrs;
    }
}

/**
 * Enable MMU with page tables at the linker-defined location

/**
 * Enable MMU with appropriate settings
 */
static void enable_mmu(void)
{
    uint64_t sctlr;
    
    /* Set Memory Attribute Indirection Register */
    uint64_t mair = (MAIR_ATTR_DEVICE_nGnRnE << (REGION_DEVICE * 8)) |
                    (MAIR_ATTR_NORMAL_NC << (REGION_NORMAL_NC * 8)) |
                    (MAIR_ATTR_NORMAL_WT << (REGION_NORMAL_WT * 8)) |
                    (MAIR_ATTR_NORMAL_WB << (REGION_NORMAL_WB * 8));
    __asm volatile("msr mair_el1, %0" : : "r" (mair));
    
    /* Set Translation Control Register */
    __asm volatile("msr tcr_el1, %0" : : "r" (TCR_EL1_VALUE));
    
    /* Point to our page tables */
    __asm volatile("msr ttbr0_el1, %0" : : "r" (l1_table));
    
    /* Ensure all previous writes are complete */
    ISB();
    
    /* Enable the MMU, I-cache, and D-cache */
    __asm volatile("mrs %0, sctlr_el1" : "=r" (sctlr));
    sctlr |= (SCTLR_M | SCTLR_I | SCTLR_C);
    __asm volatile("msr sctlr_el1, %0" : : "r" (sctlr));
    
    /* Ensure MMU is enabled before proceeding */
    ISB();
}

/**
 * Initialize the GIC-500 Interrupt Controller for Core 1
 */
static void init_gic(void)
{
    uint32_t core_id = get_current_core_id();
    
    /* Disable the distributor */
    write_reg(GICD_CTLR, 0);
    
    /* Set all interrupts to target Core 1 */
    for (int i = 32; i < 128; i++) {  /* Start with SPI interrupts */
        /* Set interrupt target to Core 1 only */
        uint32_t shift = (i % 4) * 8;
        uint32_t value = read_reg(GICD_ITARGETSR(i / 4));
        value &= ~(0xFF << shift);
        value |= (1 << (shift + core_id));
        write_reg(GICD_ITARGETSR(i / 4), value);
    }

    /* Set timer interrupt priority */
    write_reg(GICD_IPRIORITY(27), 0x80);  /* Generic Timer, middle priority */
    
    /* Configure timer interrupt */
    uint32_t reg_idx = 27 / 32;
    uint32_t bit_pos = 27 % 32;
    write_reg(GICD_ISENABLER(reg_idx), (1 << bit_pos));
    
    /* Set priority mask to accept all priorities */
    write_reg(GICC_PMR, 0xFF);
    
    /* Enable CPU interface */
    write_reg(GICC_CTLR, 1);
    
    /* Enable distributor */
    write_reg(GICD_CTLR, 1);
}

/**
 * Initialize Cache Coherent Interconnect (CCI)
 */
static void init_cci(void)
{
    volatile uint32_t *cci_ctrl = (volatile uint32_t *)(CCI_BASE + 0x0000);
    
    /* Enable cache coherency between clusters */
    *cci_ctrl |= 0x1;
}

// /**
//  * Initialize the Generic Timer with FreeRTOS tick frequency
//  */
// static void init_generic_timer(void)
// {
//     /* Set up CNTP_CTL_EL0 (Physical Timer Control register) */
//     uint64_t timer_ctrl = 1;  /* Enable timer */
//     __asm volatile("msr cntp_ctl_el0, %0" : : "r" (timer_ctrl));
    
//     /* Set timer frequency (use the system counter frequency) */
//     uint64_t cntfrq_el0;
//     __asm volatile("mrs %0, cntfrq_el0" : "=r" (cntfrq_el0));
    
//     /* Calculate timer ticks for configTICK_RATE_HZ */
//     uint64_t timer_ticks = cntfrq_el0 / configTICK_RATE_HZ;
//     uint64_t next_tick = timer_ticks;
    
//     /* Set compare value for first tick */
//     __asm volatile("msr cntp_cval_el0, %0" : : "r" (next_tick));
// }

// /**
//  * Setup FreeRTOS tick interrupt
//  */
// void FreeRTOS_SetupTickInterrupt(void)
// {
//     /* Initialize the generic timer for FreeRTOS ticks */
//     init_generic_timer();
// }

/**
 * Initialize core 1 to run FreeRTOS
 */
static void init_freertos_core(void)
{
    uint32_t core_id = get_current_core_id();
    
    /* Only run this on core 1 */
    if (core_id != 1) {
        /* This shouldn't happen, but we check anyway */
        set_core_status(core_id, CORE_ERROR);
        return;
    }
    
    /* Initialize page tables */
    init_page_tables();
    
    /* Enable MMU and caches */
    enable_mmu();
    
    /* Initialize interrupt controller */
    init_gic();
    
    /* Initialize cache coherency */
    init_cci();

    uart_init();
    
    /* Mark this core as ready to run FreeRTOS */
    set_core_status(core_id, CORE_READY);
    
    /* Signal to AT-F that FreeRTOS core is ready and it can proceed with boot */
    boot_ctrl->handoff_complete = 1;
    send_ipc_message(0, 0x1);
}

/**
 * Full system initialization
 * This is called from Reset_Handler
 */
void SystemInit(void)
{
    /* Disable interrupts */
    __asm volatile("msr DAIFSet, #0xF");
    
    uint32_t core_id = get_current_core_id();
    
    /* Initialize shared memory control structure if not already done */
    if (boot_ctrl->boot_stage == 0) {
        /* First initialization - use single check since we're early in boot */
        for (int i = 0; i < 8; i++) {
            boot_ctrl->core_status[i] = 0;
            boot_ctrl->jump_addr[i] = 0;
        }
        boot_ctrl->boot_stage = 1;
        boot_ctrl->handoff_complete = 0;
        boot_ctrl->lock = 0;
    }
    
    /* Mark core as running */
    set_core_status(core_id, CORE_RUNNING);
    
    if (core_id == 1) {
        /* Core 1 - Initialize for FreeRTOS */
        init_freertos_core();
        
        /* Enable interrupts */
        __asm volatile("msr DAIFClr, #0xF");
        /* Return to startup code which will continue with FreeRTOS init */
        print_init_complete();
        return;
    } else {
        /* For other cores */
        
        /* We didn't expect to be here on cores other than 1 */
        /* This would happen if our code gets loaded and executed on other cores */
        set_core_status(core_id, CORE_ERROR);
        
        /* Enter low-power wait state or return to bootloader */
        while (1) {
            __asm volatile("wfe");  /* Wait for event (low power) */
        }
    }
}

// /**
//  * FreeRTOS entry point - called after SystemInit
//  * This is where FreeRTOS specific initialization occurs
//  */
// void FreeRTOS_Init(void)
// {
//     uint32_t core_id = get_current_core_id();
    
//     /* Double-check we're on core 1 */
//     if (core_id != 1) {
//         while (1) { /* Should never get here */ }
//     }
    
//     /* Change status to indicate FreeRTOS is running */
//     set_core_status(core_id, CORE_FREERTOS_RUNNING);
    
//     /* Setup the FreeRTOS tick timer */
//     FreeRTOS_SetupTickInterrupt();
    
//     /* Start the first task */
//     vPortStartFirstTask();
    
//     /* Should never reach here */
//     while (1) {}
// }

/**
 * Handler for Generic Timer interrupt
 * Called from the assembly IRQ handler
 */
void FreeRTOS_Tick_Handler(void)
{
    uint32_t interrupt_id;
    BaseType_t xSwitchRequired = pdFALSE;
    
    /* Get the interrupt ID from GIC */
    interrupt_id = *((volatile uint32_t *)GICC_IAR) & 0x3FF;
    
    /* Check if this is the timer interrupt (ID 27 for Generic Timer) */
    if (interrupt_id == 27) {
        /* Increment the RTOS tick and check if a context switch is required */
        xSwitchRequired = xTaskIncrementTick();
        
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
        
        /* Signal if context switch is required */
        if (xSwitchRequired != pdFALSE) {
            ulPortYieldRequired = 1;
        }
    }
    
    /* Clear the interrupt by writing to EOIR */
    *((volatile uint32_t *)GICC_EOIR) = interrupt_id;
}

/**
 * Function to configure the Generic Timer for FreeRTOS ticks
 * Called during initialization
 */
void vConfigureTickInterrupt(void)
{
    uint64_t cntfrq_el0;
    
    /* Get counter frequency */
    __asm__ __volatile__("mrs %0, cntfrq_el0" : "=r" (cntfrq_el0));
    
    /* Calculate timer ticks for configTICK_RATE_HZ */
    uint64_t timer_ticks = cntfrq_el0 / configTICK_RATE_HZ;
    
    /* Set compare value for first tick */
    uint64_t curr_cnt;
    __asm__ __volatile__("mrs %0, cntpct_el0" : "=r" (curr_cnt));
    uint64_t next_tick = curr_cnt + timer_ticks;
    __asm__ __volatile__("msr cntp_cval_el0, %0" : : "r" (next_tick));
    
    /* Enable the timer */
    __asm__ __volatile__("msr cntp_ctl_el0, %0" : : "r" (1ULL));
    
    /* Enable the timer interrupt in the CPU's internal registers */
    uint64_t cnthctl_el2;
    __asm__ __volatile__("mrs %0, cnthctl_el2" : "=r" (cnthctl_el2));
    cnthctl_el2 |= (1 << 1);  /* Enable EL1 physical timer access */
    __asm__ __volatile__("msr cnthctl_el2, %0" : : "r" (cnthctl_el2));
}

/**
 * Implementation of FreeRTOS required function to set up tick timer
 */
void FreeRTOS_SetupTickInterrupt(void)
{
    vConfigureTickInterrupt();
}