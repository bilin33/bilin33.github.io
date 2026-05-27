#include "platform.h"

static uint16_t set_vcore_up(uint8_t level)
{
    uint32_t PMMRIE_backup, SVSMHCTL_backup, SVSMLCTL_backup;
    PMMCTL0_H = 0xA5;
    PMMRIE_backup = PMMRIE;
    PMMRIE &= ~(SVMHVLRPE | SVSHPE | SVMLVLRPE | SVSLPE | SVMHVLRIE | SVMHIE |
                SVSMHDLYIE | SVMLVLRIE | SVMLIE | SVSMLDLYIE);
    SVSMHCTL_backup = SVSMHCTL;
    SVSMLCTL_backup = SVSMLCTL;
    PMMIFG = 0;
    SVSMHCTL = SVMHE | SVSHE | (SVSMHRRL0 * level);
    while ((PMMIFG & SVSMHDLYIFG) == 0) { }
    PMMIFG &= ~SVSMHDLYIFG;
    if ((PMMIFG & SVMHIFG) == SVMHIFG) {
        PMMIFG &= ~SVSMHDLYIFG;
        SVSMHCTL = SVSMHCTL_backup;
        while ((PMMIFG & SVSMHDLYIFG) == 0) { }
        PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
        PMMRIE = PMMRIE_backup;
        PMMCTL0_H = 0x00;
        return false;
    }
    SVSMHCTL |= (SVSHRVL0 * level);
    while ((PMMIFG & SVSMHDLYIFG) == 0) { }
    PMMIFG &= ~SVSMHDLYIFG;
    PMMCTL0_L = PMMCOREV0 * level;
    SVSMLCTL = SVMLE | (SVSMLRRL0 * level) | SVSLE | (SVSLRVL0 * level);
    while ((PMMIFG & SVSMLDLYIFG) == 0) { }
    PMMIFG &= ~SVSMLDLYIFG;
    SVSMLCTL &= (SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);
    SVSMLCTL_backup &= ~(SVSLRVL0 + SVSLRVL1 + SVSMLRRL0 + SVSMLRRL1 + SVSMLRRL2);
    SVSMLCTL |= SVSMLCTL_backup;
    SVSMHCTL &= (SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);
    SVSMHCTL_backup &= ~(SVSHRVL0 + SVSHRVL1 + SVSMHRRL0 + SVSMHRRL1 + SVSMHRRL2);
    SVSMHCTL |= SVSMHCTL_backup;
    while (((PMMIFG & SVSMLDLYIFG) == 0) && ((PMMIFG & SVSMHDLYIFG) == 0)) { }
    PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
    PMMRIE = PMMRIE_backup;
    PMMCTL0_H = 0x00;
    return true;
}

static void increase_vcore_to_level2(void)
{
    uint8_t level = 2;
    uint8_t actual = PMMCTL0 & PMMCOREV_3;
    while (level > actual) {
        if (!set_vcore_up(++actual)) break;
    }
}

void clock_init_16mhz(void)
{
    increase_vcore_to_level2();
    UCSCTL3 |= SELREF_2;
    UCSCTL4 |= SELA_2;
    __bis_SR_register(SCG0);
    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_5;
    UCSCTL2 = FLLD_0 + 487;
    __bic_SR_register(SCG0);
    __delay_cycles(500000);
    do {
        UCSCTL7 &= ~(XT2OFFG | XT1LFOFFG | DCOFFG);
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
}

void gpio_init(void)
{
    /* I2C UCB0 on P2.0/P2.1 */
    P2SEL |= BIT0 | BIT1;

    /* UART UCA1 on P1.4/P1.5 */
    P1SEL |= BIT4 | BIT5;

    /* Keys active-low with pullups */
    P4DIR &= ~(KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);
    P4REN |=  (KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);
    P4OUT |=  (KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);
}

void delay_ms(uint16_t ms)
{
    while (ms--) __delay_cycles(F_CPU_HZ / 1000UL);
}

void delay_us(uint16_t us)
{
    while (us--) __delay_cycles(F_CPU_HZ / 1000000UL);
}
