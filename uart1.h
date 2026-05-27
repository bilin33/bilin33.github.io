#ifndef UART1_H_
#define UART1_H_

#include <stdint.h>
#include <stdbool.h>

void uart1_init_115200(void);
void uart1_putc(char c);
void uart1_puts(const char *s);
void uart1_send(const uint8_t *buf, uint16_t len);
void uart1_printf(const char *fmt, ...);

bool uart1_available(void);
uint8_t uart1_read_byte(void);

#endif
