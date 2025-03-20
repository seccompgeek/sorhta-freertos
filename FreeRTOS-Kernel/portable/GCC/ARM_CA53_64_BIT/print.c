#include "print.h"

/**
 * Calculate and set the baud rate generator registers
 */
static void linflex_set_brg(uint32_t clock, uint32_t baud)
{
    volatile uint32_t *linibrr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_LINIBRR);
    volatile uint32_t *linfbrr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_LINFBRR);
    volatile uint32_t *uartcr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTCR);
    uint32_t ldiv_mult = LDIV_MULTIPLIER;
    uint32_t ibr, fbr;

    /* Check if Reduced Oversampling is enabled */
    uint32_t cr_val = *uartcr;
    if (cr_val & UARTCR_ROSE) {
        /* Extract OSR field if ROSE is set */
        ldiv_mult = (cr_val >> 24) & 0xF;
    }

    /* Calculate integer and fractional dividers */
    uint32_t dividr = baud * ldiv_mult;
    uint32_t divisr = clock;
    
    ibr = divisr / dividr;
    fbr = ((divisr % dividr) * 16) / dividr;
    fbr &= 0xF;

    /* Set the baud rate registers */
    *linibrr = ibr;
    *linfbrr = fbr;
}

/**
 * Initialize the LinFLEX UART for console output
 */
void uart_init(void)
{
    volatile uint32_t *lincr1 = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_LINCR1);
    volatile uint32_t *linsr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_LINSR);
    volatile uint32_t *uartcr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTCR);
    volatile uint32_t *uartpto = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTPTO);
    
    /* Set master mode and init mode */
    *lincr1 = LINCR1_INIT;
    *lincr1 = LINCR1_MME | LINCR1_INIT;
    
    /* Wait for init mode entry */
    while ((*linsr & LINSR_LINS_MASK) != LINSR_LINS_INITMODE) {
        /* Wait */
    }
    
    /* Set UART bit */
    *uartcr = UARTCR_UART;
    
    /* Set baud rate */
    linflex_set_brg(UART_CLOCK_HZ, UART_BAUD_RATE);
    
    /* Set preset timeout register value */
    *uartpto = 0xF;
    
    /* 8-bit data, no parity, Tx/Rx enabled, UART mode, FIFO mode */
    *uartcr = UARTCR_PC1 | UARTCR_RXEN | UARTCR_TXEN | UARTCR_PC0 | 
              UARTCR_WL0 | UARTCR_UART | UARTCR_RFBM | UARTCR_TFBM;
    
    /* End init mode */
    *lincr1 &= ~LINCR1_INIT;
}

/**
 * Wait for the transmit buffer to be empty
 */
static void uart_wait_tx_complete(void)
{
    volatile uint32_t *uartcr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTCR);
    volatile uint32_t *uartsr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTSR);
    
    /* Check if FIFO mode or buffer mode */
    uint32_t is_fifo_mode = *uartcr & UARTCR_TFBM;
    
    if (is_fifo_mode) {
        /* FIFO mode - wait for DTF flag to clear */
        while (*uartsr & UARTSR_DTF) {
            /* Wait */
        }
    } else {
        /* Buffer mode - wait for DTF flag to set, then clear it */
        while (!(*uartsr & UARTSR_DTF)) {
            /* Wait */
        }
        *uartsr = UARTSR_DTF;  /* Clear the flag in buffer mode */
    }
}

/**
 * Send a single character to UART
 */
void uart_putc(char c)
{
    volatile uint32_t *bdrl = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_BDRL);
    volatile uint32_t *uartcr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTCR);
    volatile uint32_t *uartsr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTSR);
    
    /* If it's a newline, send carriage return first */
    if (c == '\n') {
        uart_putc('\r');
    }
    
    /* Check if FIFO mode or buffer mode */
    uint32_t is_fifo_mode = *uartcr & UARTCR_TFBM;
    
    if (is_fifo_mode) {
        /* FIFO mode - wait for DTF flag to clear */
        while (*uartsr & UARTSR_DTF) {
            /* Wait */
        }
    }
    
    /* Write character to data register */
    *bdrl = c;
    
    if (!is_fifo_mode) {
        /* Buffer mode - wait for DTF flag to set, then clear it */
        while (!(*uartsr & UARTSR_DTF)) {
            /* Wait */
        }
        *uartsr = UARTSR_DTF;  /* Clear the flag in buffer mode */
    }
}

/**
 * Flush the transmit buffer
 */
void uart_flush(void)
{
    volatile uint32_t *uartcr = (volatile uint32_t *)(LINFLEX_BASE + LINFLEX_UARTCR);
    
    /* Check if FIFO mode or buffer mode */
    uint32_t is_fifo_mode = *uartcr & UARTCR_TFBM;
    
    if (is_fifo_mode) {
        /* In FIFO mode, wait until TFC counter is zero */
        while ((*uartcr & UARTCR_TFC) != 0) {
            /* Wait */
        }
    } else {
        /* In buffer mode, just ensure the last character was sent */
        uart_wait_tx_complete();
    }
}

/**
 * Send a string to UART
 */
void uart_puts(const char *str)
{
    while (*str) {
        uart_putc(*str++);
    }
    uart_flush();
}

/**
 * Print a hexadecimal value
 */
void uart_print_hex(uint32_t value)
{
    const char hex_chars[] = "0123456789ABCDEF";
    char buffer[11];  /* "0x" + 8 hex digits + null terminator */
    int i;
    
    buffer[0] = '0';
    buffer[1] = 'x';
    buffer[10] = '\0';
    
    for (i = 9; i >= 2; i--) {
        buffer[i] = hex_chars[value & 0xF];
        value >>= 4;
    }
    
    uart_puts(buffer);
}

/**
 * Print initialization complete message
 */
void print_init_complete(void)
{
    const char *msg = "\n\n"
                      "**********************************************\n"
                      "*                                            *\n"
                      "*  S32G3 FreeRTOS System Initialization      *\n"
                      "*  Successfully Completed                    *\n"
                      "*                                            *\n"
                      "*  Core 1 is now running FreeRTOS            *\n"
                      "*  Core 0 has returned to AT-F               *\n"
                      "*                                            *\n"
                      "**********************************************\n\n";
    
    uart_puts(msg);
}

/**
 * Print a diagnostic message during initialization
 */
void print_init_message(const char *message)
{
    uart_puts("[INIT] ");
    uart_puts(message);
    uart_puts("\n");
}

/**
 * Print core status information
 */
void print_core_status(uint32_t core_id, const char *status)
{
    char core_char = '0' + core_id;
    
    uart_puts("Core ");
    uart_putc(core_char);
    uart_puts(": ");
    uart_puts(status);
    uart_puts("\n");
}

/**
 * Example usage:
 * 
 * void main(void)
 * {
 *     uart_init();
 *     print_init_message("Starting system initialization");
 *     
 *     // System initialization code
 *     
 *     print_core_status(0, "Initializing core handoff");
 *     print_core_status(1, "Starting FreeRTOS");
 *     
 *     // Complete initialization
 *     print_init_complete();
 * }
 */