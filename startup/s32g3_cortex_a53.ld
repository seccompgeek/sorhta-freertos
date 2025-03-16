/* Linker script for S32G3 Cortex-A53 */
MEMORY
{
    DDR_RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 512M
}

ENTRY(Reset_Handler)

SECTIONS
{
        .text.startup :
    {
        KEEP(*(.text.startup))
        . = ALIGN(8);
    } > DDR_RAM

    .vectors :
    {
        . = ALIGN(4096);
        KEEP(*(.vectors))
        . = ALIGN(4096);
    } > DDR_RAM

    .text :
    {
        . = ALIGN(8);
        *(.text.Reset_Handler)  /* Place Reset_Handler first */
        *(.text)
        *(.text*)
        
        /* Add specific sections for constructor and destructor tables */
        KEEP(*(.init))
        KEEP(*(.fini))
        
        /* Add specific section for exception handling */
        *(.eh_frame)
        
        /* Align here to ensure _etext is properly aligned */
        . = ALIGN(8);
        _etext = .;
    } > DDR_RAM
    
    /* Add a separate read-only data section */
    .rodata :
    {
        . = ALIGN(8);
        *(.rodata)
        *(.rodata*)
        *(.rodata.*)
        . = ALIGN(8);
    } > DDR_RAM

    /* Add space for translation tables */
    .ttb :
    {
        . = ALIGN(4096);  /* Translation tables must be 4KB aligned */
        _ttb0_base = .;
        . = . + 0x1000;   /* Level 1 table for TTBR0 (4KB) */
        . = ALIGN(4096);
    } > DDR_RAM

    .data :
    {
        . = ALIGN(8);
        _sdata = .;
        *(.data)
        *(.data*)
        
        /* Add specific section for constructors/destructors */
        KEEP(*(.ctors))
        KEEP(*(.dtors))
        
        . = ALIGN(8);
        _edata = .;
    } > DDR_RAM

    .bss :
    {
        . = ALIGN(8);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
        _end = .;
    } > DDR_RAM

    .stack :
    {
        . = ALIGN(16);
        _stack_bottom = .;
        . = . + 0x10000;  /* 64KB stack */
        . = ALIGN(16);    /* Stack should be 16-byte aligned for AArch64 */
        _stack_top = .;
    } > DDR_RAM

    .heap :
    {
        . = ALIGN(16);
        __heap_start__ = .;
        . = . + 0x1000000;  /* 16MB heap */
        __heap_end__ = .;
    } > DDR_RAM

    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}