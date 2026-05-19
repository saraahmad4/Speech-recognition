#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "lcd_helpers.h"

// ============================================================
// LCD Pulse and Byte Write
// ============================================================
void lcd_pulse_enable(void) {
    LCD_CTRL_PORT |= (1 << LCD_EN);
    _delay_us(1);
    LCD_CTRL_PORT &= ~(1 << LCD_EN);
    _delay_us(100);
}

void lcd_write_byte(uint8_t value, bool command) {
    if (command) {
        LCD_CTRL_PORT &= ~(1 << LCD_RS);
    } else {
        LCD_CTRL_PORT |= (1 << LCD_RS);
    }
    LCD_CTRL_PORT &= ~(1 << LCD_RW);
    LCD_DATA_PORT = value;
    _delay_us(1);
    lcd_pulse_enable();
}

// ============================================================
// LCD Initialization
// ============================================================
void lcd_init(void) {
    LCD_DATA_DDR = 0xFF;
    LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_EN);
    _delay_ms(40);
    lcd_write_byte(0x38, true);
    _delay_ms(5);
    lcd_write_byte(0x38, true);
    _delay_us(150);
    lcd_write_byte(0x38, true);
    lcd_write_byte(0x0C, true);
    lcd_write_byte(0x06, true);
    lcd_write_byte(0x01, true);
    _delay_ms(2);
}

// ============================================================
// LCD Control Commands
// ============================================================
void lcd_clear(void) {
    lcd_write_byte(0x01, true);
    _delay_ms(5);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    address += col;
    lcd_write_byte(0x80 | address, true);
}

// ============================================================
// LCD Text Output
// ============================================================
void lcd_print(const char *str) {
    while (*str) {
        lcd_write_byte((uint8_t)*str++, false);
    }
}

void lcd_print_float(float value, uint8_t decimals)
{
    if (value < 0.0f) {
        lcd_write_byte('-', false);
        value = -value;
    }

    uint16_t integer = (uint16_t)value;
    uint16_t temp = integer;

    // Print integer part
    char buf[6];
    uint8_t idx = 0;

    if (temp == 0) {
        lcd_write_byte('0', false);
    } else {
        while (temp > 0 && idx < sizeof(buf)) {
            buf[idx++] = '0' + (temp % 10);
            temp /= 10;
        }

        for (int8_t i = idx - 1; i >= 0; i--) {
            lcd_write_byte(buf[i], false);
        }
    }

    // Print decimal point
    lcd_write_byte('.', false);

    // Fractional part
    float remainder = value - (float)integer;

    for (uint8_t i = 0; i < decimals; i++) {
        remainder *= 10.0f;

        uint8_t digit = (uint8_t)remainder;

        lcd_write_byte('0' + digit, false);

        remainder -= digit;
    }
}
