#ifndef _UART_HELPERS
#define _UART_HELPERS

#include <stdint.h>
#include <stdbool.h>

// UART Configuration
#define UART_BAUD 9600UL
#define UART_CMD_BUFFER_SIZE 32

// ============================================================
// UART Initialization and Basic I/O
// ============================================================
void uart_init(void);
void uart_transmit(uint8_t data);
void uart_puts(const char *str);

// ============================================================
// UART Number Printing
// ============================================================
void uart_print_uint16(uint16_t value);
void uart_print_float(float value, uint8_t decimals);

// ============================================================
// UART Input/Command Interface
// ============================================================
uint8_t uart_receive(void);
bool uart_data_available(void);
void uart_process_input(void);
void handle_uart_command(const char *cmd);

// ============================================================
// External Global Variables
// ============================================================
extern char uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
extern uint8_t uart_cmd_index;

#endif // UART_H
