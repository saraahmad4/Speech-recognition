#define F_CPU 11059200UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "uart_helpers.h"

// ============================================================
// UART Command Interface - Global Variables
// ============================================================
char uart_cmd_buffer[UART_CMD_BUFFER_SIZE];
uint8_t uart_cmd_index = 0;

// ============================================================
// Forward Declarations
// ============================================================
void reset_capture_state(void);
void lcd_clear(void);
void lcd_print(const char *str);
void calibrate_noise_floor(void);
void start_recognition(void);

// ============================================================
// UART Initialization and Basic I/O
// ============================================================
void uart_init(void) {
    uint16_t ubrr = (F_CPU / (16UL * UART_BAUD)) - 1;
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;
    UCSRB = (1 << TXEN) | (1 << RXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

void uart_transmit(uint8_t data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

void uart_puts(const char *str) {
    while (*str) {
        uart_transmit((uint8_t)*str++);
    }
}

// ============================================================
// UART Number Printing
// ============================================================
void uart_print_uint16(uint16_t value) {
    char buf[6];
    uint8_t idx = 0;
    if (value == 0) {
        uart_transmit('0');
        return;
    }
    while (value > 0 && idx < sizeof(buf)) {
        buf[idx++] = '0' + (value % 10);
        value /= 10;
    }
    for (int8_t i = idx - 1; i >= 0; i--) {
        uart_transmit(buf[i]);
    }
}

void uart_print_float(float value, uint8_t decimals) {
    if (value < 0.0f) {
        uart_transmit('-');
        value = -value;
    }
    uint16_t integer = (uint16_t)value;
    uart_print_uint16(integer);
    uart_transmit('.');
    float remainder = value - (float)integer;
    for (uint8_t i = 0; i < decimals; i++) {
        remainder *= 10.0f;
        uint8_t digit = (uint8_t)remainder;
        uart_transmit('0' + digit);
        remainder -= digit;
    }
}

// ============================================================
// UART Input/Command Interface
// ============================================================
uint8_t uart_receive(void) {
    while (!(UCSRA & (1 << RXC)));
    return UDR;
}

bool uart_data_available(void) {
    return (UCSRA & (1 << RXC)) != 0;
}

void uart_process_input(void) {
    if (uart_data_available()) {
        uint8_t ch = uart_receive();
        
        // Handle backspace
        if (ch == 8 || ch == 127) {
            if (uart_cmd_index > 0) {
                uart_cmd_index--;
                uart_puts("\b \b");
            }
            return;
        }
        
        // Handle newline/carriage return
        if (ch == '\r' || ch == '\n') {
            uart_transmit('\r');
            uart_transmit('\n');
            uart_cmd_buffer[uart_cmd_index] = '\0';
            
            // Process the command
            if (uart_cmd_index > 0) {
                handle_uart_command(uart_cmd_buffer);
            }
            
            uart_cmd_index = 0;
            return;
        }
        
        // Store printable characters
        if (ch >= 32 && ch < 127 && uart_cmd_index < UART_CMD_BUFFER_SIZE - 1) {
            uart_cmd_buffer[uart_cmd_index++] = ch;
        }
    }
}

void handle_uart_command(const char *cmd) {
    // Skip leading spaces
    uint8_t idx = 0;
    while (cmd[idx] == ' ' && cmd[idx] != '\0') idx++;
    
    const char *start = &cmd[idx];
    uint8_t cmd_len = 0;
    
    // Find command length (until space or null terminator)
    while (start[cmd_len] != ' ' && start[cmd_len] != '\0') {
        cmd_len++;
    }
    
    // Convert to uppercase for comparison
    char cmd_upper[32];
    for (uint8_t i = 0; i < cmd_len && i < 31; i++) {
        if (start[i] >= 'a' && start[i] <= 'z') {
            cmd_upper[i] = start[i] - 'a' + 'A';
        } else {
            cmd_upper[i] = start[i];
        }
    }
    cmd_upper[cmd_len] = '\0';
    
    // Process commands
    if (cmd_len == 5 && cmd_upper[0] == 'R' && cmd_upper[1] == 'E' && 
        cmd_upper[2] == 'S' && cmd_upper[3] == 'E' && cmd_upper[4] == 'T') {
        // RESET command
        uart_puts("[UART] Executing RESET\r\n");
        reset_capture_state();
        lcd_clear();
        lcd_print("Ready");
        uart_puts("[UART] RESET complete\r\n");
    } 
    else if (cmd_len == 5 && cmd_upper[0] == 'N' && cmd_upper[1] == 'O' && 
             cmd_upper[2] == 'I' && cmd_upper[3] == 'S' && cmd_upper[4] == 'E' ) {
        // CAPTURE NOISE command
        uart_puts("[UART] Executing CAPTURE NOISE\r\n");
        calibrate_noise_floor();
    } 
    else if (cmd_len == 5 && cmd_upper[0] == 'S' && cmd_upper[1] == 'T' && 
             cmd_upper[2] == 'A' && cmd_upper[3] == 'R' && cmd_upper[4] == 'T') {
        // START command
        uart_puts("[UART] Executing START\r\n");
        start_recognition();
    } 
    else {
        // Invalid command
        uart_puts("[UART] Invalid command: ");
        uart_puts(start);
        uart_puts("\r\n");
    }
}
