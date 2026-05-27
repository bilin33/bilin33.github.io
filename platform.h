#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#define F_CPU_HZ 16000000UL

/* User wiring
 * I2C:  P2.0 UCB0SCL, P2.1 UCB0SDA  (enable both P2SEL bits)
 * UART: P1.4 UCA1RXD, P1.5 UCA1TXD  -> CH340E
 * Keys: P4.6 KEY+, P4.4 KEY-, P4.2 KEYSW, active low with pullups
 */
#define KEY_PLUS_BIT   BIT6
#define KEY_MINUS_BIT  BIT4
#define KEY_SW_BIT     BIT2

void clock_init_16mhz(void);
void gpio_init(void);
void delay_ms(uint16_t ms);
void delay_us(uint16_t us);

#endif
