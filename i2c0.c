#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c0.h"

/* UCB0 I2C driver copied to the same interrupt/LPM style as your working
 * single-file project. This replaces the polling I2C code because BME280 and
 * DAC80502 work with this state machine on your board.
 */
#define I2C_MAX_BUFFER_SIZE 64u

#define I2C_TIMEOUT_SPINS 60000u

static bool i2c0_wait_bus_idle(void)
{
    uint16_t t = I2C_TIMEOUT_SPINS;
    while ((UCB0STAT & UCBBUSY) && t--) { }
    return t != 0u;
}

static bool i2c0_wait_stop_done(void)
{
    uint16_t t = I2C_TIMEOUT_SPINS;
    while ((UCB0CTLW0 & UCTXSTP) && t--) { }
    return t != 0u;
}


typedef enum {
    I2C_IDLE_MODE,
    I2C_NACK_MODE,
    I2C_TX_REG_ADDRESS_MODE,
    I2C_TX_DATA_MODE,
    I2C_RX_DATA_MODE,
    I2C_SWITCH_TO_RX_MODE
} I2C_Mode;

static volatile I2C_Mode MasterMode = I2C_IDLE_MODE;
static volatile uint8_t TransmitRegAddr = 0;
static volatile uint8_t ReceiveBuffer[I2C_MAX_BUFFER_SIZE];
static volatile uint8_t RXByteCtr = 0;
static volatile uint8_t ReceiveIndex = 0;
static volatile uint8_t TransmitBuffer[I2C_MAX_BUFFER_SIZE];
static volatile uint8_t TXByteCtr = 0;
static volatile uint8_t TransmitIndex = 0;

static void copy_to_tx(const uint8_t *src, uint8_t count)
{
    uint8_t i;
    for (i = 0; i < count; i++) TransmitBuffer[i] = src[i];
}

void i2c0_init_100khz(void)
{
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 |= UCMST | UCMODE_3 | UCSYNC | UCSSEL_2;
    UCB0BRW_L = 160;
    UCB0BRW_H = 0;
    UCB0CTLW0 &= ~UCSWRST;
    UCB0IE = UCNACKIE;
}

bool i2c0_write(uint8_t addr, const uint8_t *data, uint16_t len)
{
    /* Raw write: first byte in data is sent as the first I2C data byte. */
    uint8_t i;
    if (len == 0 || len > I2C_MAX_BUFFER_SIZE) return false;
    if (!i2c0_wait_stop_done() || !i2c0_wait_bus_idle()) return false;

    for (i = 0; i < (uint8_t)len; i++) TransmitBuffer[i] = data[i];
    TXByteCtr = (uint8_t)len;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;
    MasterMode = I2C_TX_DATA_MODE;

    UCB0I2CSA = addr;
    UCB0IFG &= ~(UCTXIFG | UCRXIFG | UCNACKIFG);
    UCB0IE &= ~UCRXIE;
    UCB0IE |= UCTXIE | UCNACKIE;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    __bis_SR_register(LPM0_bits | GIE);
    if (!i2c0_wait_stop_done()) return false;

    return MasterMode == I2C_IDLE_MODE;
}

bool i2c0_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len)
{
    if (len > I2C_MAX_BUFFER_SIZE) return false;
    if (!i2c0_wait_stop_done() || !i2c0_wait_bus_idle()) return false;

    MasterMode = I2C_TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg;
    copy_to_tx(data, (uint8_t)len);
    TXByteCtr = (uint8_t)len;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    UCB0I2CSA = addr;
    UCB0IFG &= ~(UCTXIFG | UCRXIFG | UCNACKIFG);
    UCB0IE &= ~UCRXIE;
    UCB0IE |= UCTXIE | UCNACKIE;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    __bis_SR_register(LPM0_bits | GIE);
    if (!i2c0_wait_stop_done()) return false;

    return MasterMode == I2C_IDLE_MODE;
}

bool i2c0_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t i;
    if (len == 0 || len > I2C_MAX_BUFFER_SIZE) return false;
    if (!i2c0_wait_stop_done() || !i2c0_wait_bus_idle()) return false;

    MasterMode = I2C_TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg;
    RXByteCtr = (uint8_t)len;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    UCB0I2CSA = addr;
    UCB0IFG &= ~(UCTXIFG | UCRXIFG | UCNACKIFG);
    UCB0IE &= ~UCRXIE;
    UCB0IE |= UCTXIE | UCNACKIE;
    UCB0CTLW0 |= UCTR | UCTXSTT;
    __bis_SR_register(LPM0_bits | GIE);
    if (!i2c0_wait_stop_done()) return false;

    if (MasterMode != I2C_IDLE_MODE) return false;
    for (i = 0; i < (uint8_t)len; i++) data[i] = ReceiveBuffer[i];
    return true;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t rx_val = 0;

    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE: break;
        case USCI_I2C_UCALIFG: break;

        case USCI_I2C_UCNACKIFG:
            UCB0CTLW0 |= UCTXSTP;
            UCB0IFG &= ~UCNACKIFG;
            MasterMode = I2C_NACK_MODE;
            UCB0IE &= ~(UCTXIE | UCRXIE);
            __bic_SR_register_on_exit(CPUOFF);
            break;

        case USCI_I2C_UCSTTIFG: break;
        case USCI_I2C_UCSTPIFG: break;
        case USCI_I2C_UCRXIFG3: break;
        case USCI_I2C_UCTXIFG3: break;
        case USCI_I2C_UCRXIFG2: break;
        case USCI_I2C_UCTXIFG2: break;
        case USCI_I2C_UCRXIFG1: break;
        case USCI_I2C_UCTXIFG1: break;

        case USCI_I2C_UCRXIFG0:
            rx_val = UCB0RXBUF;
            if (RXByteCtr) {
                ReceiveBuffer[ReceiveIndex++] = rx_val;
                RXByteCtr--;
            }
            if (RXByteCtr == 1) {
                UCB0CTLW0 |= UCTXSTP;
            } else if (RXByteCtr == 0) {
                UCB0IE &= ~UCRXIE;
                MasterMode = I2C_IDLE_MODE;
                __bic_SR_register_on_exit(CPUOFF);
            }
            break;

        case USCI_I2C_UCTXIFG0:
            switch (MasterMode)
            {
                case I2C_TX_REG_ADDRESS_MODE:
                    UCB0TXBUF = TransmitRegAddr;
                    if (RXByteCtr) MasterMode = I2C_SWITCH_TO_RX_MODE;
                    else MasterMode = I2C_TX_DATA_MODE;
                    break;

                case I2C_SWITCH_TO_RX_MODE:
                    UCB0IE |= UCRXIE;
                    UCB0IE &= ~UCTXIE;
                    UCB0CTLW0 &= ~UCTR;
                    MasterMode = I2C_RX_DATA_MODE;
                    UCB0CTLW0 |= UCTXSTT;
                    if (RXByteCtr == 1) {
                        while (UCB0CTLW0 & UCTXSTT) { }
                        UCB0CTLW0 |= UCTXSTP;
                    }
                    break;

                case I2C_TX_DATA_MODE:
                    if (TXByteCtr) {
                        UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                        TXByteCtr--;
                    } else {
                        UCB0CTLW0 |= UCTXSTP;
                        MasterMode = I2C_IDLE_MODE;
                        UCB0IE &= ~UCTXIE;
                        __bic_SR_register_on_exit(CPUOFF);
                    }
                    break;

                default:
                    __no_operation();
                    break;
            }
            break;

        default: break;
    }
}
