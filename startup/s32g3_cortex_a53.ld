/* Linker script for S32G3 Cortex-A53 */
MEMORY
{
    DDR_RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 512M
}

ENTRY(Reset_Handler)

SECTIONS
{
    .vectors :
    {
        . = ALIGN(4096);
        KEEP(*(.vectors))
        . = ALIGN(4096);
    } > DDR_RAM

    .text :
    {
        . = ALIGN(8);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(8);
        _etext = .;
    } > DDR_RAM

    .data :
    {
        . = ALIGN(8);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(8);
        _edata = .;
    } > DDR_RAM

    /* In s32g3_cortex_a53.ld */
    .bss :
    {
        . = ALIGN(8);
        _sbss = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
        _end = .; /* Add this line */
    } > DDR_RAM

    .stack :
    {
        . = ALIGN(16);
        _stack_bottom = .;
        . = . + 0x10000;  /* 64KB stack */
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