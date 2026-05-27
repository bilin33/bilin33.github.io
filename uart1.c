#include "platform.h"
#include "uart1.h"
#include <stdarg.h>
#include <stdio.h>

#define UART_RX_BUF_SIZE 256U

static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

void uart1_init_115200(void)
{
    UCA1CTLW0 = UCSWRST;
    UCA1CTLW0 |= UCSSEL_2;            /* SMCLK = 16 MHz */

    /* 115200 baud @ 16 MHz, oversampling mode. */
    UCA1BRW = 8;
    UCA1MCTLW = UCOS16 | UCBRF_10 | 0xF700;

    UCA1CTLW0 &= ~UCSWRST;
    UCA1IE |= UCRXIE;                 /* Enable RX interrupt */
}

void uart1_putc(char c)
{
    while (!(UCA1IFG & UCTXIFG)) { }
    UCA1TXBUF = (uint8_t)c;
}

void uart1_puts(const char *s)
{
    while (*s) uart1_putc(*s++);
}

void uart1_send(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        while (!(UCA1IFG & UCTXIFG)) { }
        UCA1TXBUF = buf[i];
    }
}

bool uart1_available(void)
{
    return (rx_head != rx_tail);
}

uint8_t uart1_read_byte(void)
{
    uint8_t b;
    b = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_BUF_SIZE);
    return b;
}

void uart1_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart1_puts(buf);
}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
    switch (__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_UART_UCRXIFG:
    {
        uint16_t next;
        next = (uint16_t)((rx_head + 1U) % UART_RX_BUF_SIZE);
        if (next != rx_tail) {         /* drop new byte if buffer is full */
            rx_buf[rx_head] = UCA1RXBUF;
            rx_head = next;
        } else {
            (void)UCA1RXBUF;
        }
        break;
    }
    default:
        break;
    }
}
