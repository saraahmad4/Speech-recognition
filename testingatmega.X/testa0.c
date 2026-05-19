/*
 * ATmega32A - Test PORTA.0 with an LED
 *
 * Connect:
 * LED anode  -> PA0 through 220? resistor
 * LED cathode -> GND
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    // Set PA0 as output
    DDRA |= (1 << PA0);

    while (1)
    {
        // Turn LED ON
        PORTA |= (1 << PA0);
        _delay_ms(500);

        // Turn LED OFF
        PORTA &= ~(1 << PA0);
        _delay_ms(500);
    }

    return 0;
}
