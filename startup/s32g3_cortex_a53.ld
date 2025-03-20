/* Linker script for S32G3 Cortex-A53 FreeRTOS with fixed memory layout */
MEMORY
{
    /* Define memory regions with increased sizes */
    CODE_REGION (rx)  : ORIGIN = 0xE0000000, LENGTH = 0x80000    /* 512KB for code */
    BSS_REGION  (rw)  : ORIGIN = 0xE0080000, LENGTH = 0x200000   /* 2MB for BSS - increased */
    DATA_REGION (rwx) : ORIGIN = 0xE0280000, LENGTH = 0x100000   /* 1MB for other data */
    PAGE_TABLES (rwx) : ORIGIN = 0xE0380000, LENGTH = 0x20000    /* 128KB for page tables */
    HEAP_REGION (rwx) : ORIGIN = 0xE0400000, LENGTH = 0x100000   /* 1MB for heap */
    STACK_REGION (rwx): ORIGIN = 0xE0500000, LENGTH = 0x10000    /* 64KB for stack */
}

ENTRY(_Reset)

/* Define global symbols for memory region addresses */
__code_region_start = ORIGIN(CODE_REGION);
__code_region_end = ORIGIN(CODE_REGION) + LENGTH(CODE_REGION);
__bss_region_start = ORIGIN(BSS_REGION);
__bss_region_end = ORIGIN(BSS_REGION) + LENGTH(BSS_REGION);
__data_region_start = ORIGIN(DATA_REGION);
__data_region_end = ORIGIN(DATA_REGION) + LENGTH(DATA_REGION);
__page_tables_start = ORIGIN(PAGE_TABLES);
__page_tables_end = ORIGIN(PAGE_TABLES) + LENGTH(PAGE_TABLES);
__heap_region_start = ORIGIN(HEAP_REGION);
__heap_region_end = ORIGIN(HEAP_REGION) + LENGTH(HEAP_REGION);
__stack_region_start = ORIGIN(STACK_REGION);
__stack_region_end = ORIGIN(STACK_REGION) + LENGTH(STACK_REGION);

/* Define absolute symbols for stack */
_stack_bottom = __stack_region_start;
_stack_top = __stack_region_end;

/* Define heap boundaries */
__heap_start = __heap_region_start;
__heap_end = __heap_region_end;

/* FreeRTOS specific heap symbols */
__freertos_heap_start = __heap_start;
__freertos_heap_size = __heap_end - __heap_start;

SECTIONS
{
    PROVIDE(_stack_top = ORIGIN(STACK_REGION) + LENGTH(STACK_REGION));
    /* Code section - placed in CODE_REGION */
    .text :
    {
        KEEP(*(.text.startup))
        . = ALIGN(16);
        
        /* Vector table - critical for ARM */
        KEEP(*(.vectors))
        . = ALIGN(16);
        
        *(.text.Reset_Handler)  /* Place Reset_Handler first */
        *(.text)
        *(.text*)
        
        /* Special code sections */
        KEEP(*(.init))
        KEEP(*(.fini))
        
        /* Exception handling */
        *(.eh_frame)
        
        /* Read-only data */
        . = ALIGN(8);
        *(.rodata)
        *(.rodata*)
        
        . = ALIGN(8);
        _etext = .;
    } > CODE_REGION
    
    /* Data section in its own region */
    .data :
    {
        . = ALIGN(8);
        _sdata = .;
        *(.data)
        *(.data*)
        KEEP(*(.ctors))
        KEEP(*(.dtors))
        . = ALIGN(8);
        _edata = .;
    } > DATA_REGION AT > CODE_REGION
    
    /* BSS section in its own dedicated region */
    .bss (NOLOAD) :
    {
        . = ALIGN(8);
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
    } > BSS_REGION
    
    /* FreeRTOS specific data */
    .freertos_data (NOLOAD) :
    {
        . = ALIGN(8);
        *(.freertos.data)
        . = ALIGN(8);
    } > DATA_REGION
    
    /* Page tables in dedicated region */
    .page_tables (NOLOAD) :
    {
        . = ALIGN(4096);  /* Page tables must be 4KB aligned */
        *(.page_tables)
        . = ALIGN(4096);
    } > PAGE_TABLES
    
    _end = .;

    /* Discard unnecessary sections */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}