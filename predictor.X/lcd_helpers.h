#ifndef _LCD_HELPERS
#define _LCD_HELPERS

#include <stdint.h>
#include <stdbool.h>

// LCD pins (8-bit mode)
#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR DDRC
#define LCD_CTRL_PORT PORTD
#define LCD_CTRL_DDR DDRD
#define LCD_RS PD3
#define LCD_RW PD4
#define LCD_EN PD5

// ============================================================
// LCD Initialization and Control
// ============================================================
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);

// ============================================================
// LCD Text Output
// ============================================================
void lcd_print(const char *str);
void lcd_print_float(float value, uint8_t decimals);

// ============================================================
// LCD Internal (Low-level)
// ============================================================
void lcd_pulse_enable(void);
void lcd_write_byte(uint8_t value, bool command);

#endif // LCD_H
