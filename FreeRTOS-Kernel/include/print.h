#include <stdint.h>
#include <stddef.h>

/* S32G3 LinFLEX UART register definitions */
#define LINFLEX_BASE            0x401C8000  /* Adjust based on your specific UART instance */
#define UART0_BASE              (LINFLEX_BASE)

/* LinFLEX register offsets */
#define LINFLEX_LINCR1          0x00  /* Control Register 1 */
#define LINFLEX_LINSR           0x08  /* Status Register */
#define LINFLEX_UARTCR          0x10  /* UART Control Register */
#define LINFLEX_UARTSR          0x14  /* UART Status Register */
#define LINFLEX_LINFBRR         0x24  /* Fractional Baud Rate Register */
#define LINFLEX_LINIBRR         0x28  /* Integer Baud Rate Register */
#define LINFLEX_BDRL            0x38  /* Buffer Data Register Least Significant */
#define LINFLEX_UARTPTO         0x50  /* UART Preset Timeout Register */

/* LinFLEX register bit definitions */
#define LINCR1_INIT             (1 << 0)  /* Initialization Mode */
#define LINCR1_MME              (1 << 4)  /* Master Mode Enable */

#define LINSR_LINS_MASK         0x0000F000
#define LINSR_LINS_INITMODE     0x00001000
#define LINSR_LINS_RX_TX_MODE   0x00008000

#define UARTCR_UART             (1 << 0)  /* UART Mode */
#define UARTCR_WL0              (1 << 1)  /* Word Length bit 0 */
#define UARTCR_PC0              (1 << 3)  /* Parity Control bit 0 */
#define UARTCR_TXEN             (1 << 4)  /* Transmitter Enable */
#define UARTCR_RXEN             (1 << 5)  /* Receiver Enable */
#define UARTCR_PC1              (1 << 6)  /* Parity Control bit 1 */
#define UARTCR_TFBM             (1 << 8)  /* TX FIFO/Buffer Mode */
#define UARTCR_RFBM             (1 << 9)  /* RX FIFO/Buffer Mode */
#define UARTCR_TFC              (0x7 << 13) /* Transmit FIFO Counter */
#define UARTCR_ROSE             (1 << 23)  /* Reduced Oversampling Enable */

#define UARTSR_DTF              (1 << 1)   /* Data Transmission Completed Flag */

/* UART settings */
#define UART_CLOCK_HZ           40000000   /* 40MHz - adjust for your clock */
#define UART_BAUD_RATE          115200     /* 115200 baud */
#define LDIV_MULTIPLIER         16


/**
 * Initialize the LinFLEX UART for console output
 */
void uart_init(void);

/**
 * Send a single character to UART
 */
void uart_putc(char c);

/**
 * Flush the transmit buffer
 */
void uart_flush(void);

/**
 * Send a string to UART
 */
void uart_puts(const char *str);
/**
 * Print a hexadecimal value
 */
void uart_print_hex(uint32_t value);

/**
 * Print initialization complete message
 */
void print_init_complete(void);

/**
 * Print a diagnostic message during initialization
 */
void print_init_message(const char *message);

/**
 * Print core status information
 */
void print_core_status(uint32_t core_id, const char *status);

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