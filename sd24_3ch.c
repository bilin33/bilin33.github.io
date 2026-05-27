#include "platform.h"
#include "sd24_3ch.h"

#define SD24_TIMEOUT_COUNT  1200000UL

/* MSP430F6736A SD24_B, 3-channel group conversion.
 * Hardware connections:
 *   CH0: SD0P0 - SD0N0 = VOUTWE  - AGND
 *   CH1: SD1P0 - SD1N0 = BIAS    - TIAVOUT
 *   CH2: SD2P0 - SD2N0 = AGND    - VOUTRE
 *
 * TI's 3-channel example relies on the power-on default input selection,
 * which is Px0/Nx0 for each SD24 converter. Therefore no SD24INCH_0 macro
 * is required for this device header.
 */
#define SD24_VREF_MV          1200L
#define SD24_FULL_SCALE_16    32768L      /* signed 16-bit result from SD24BMEMHn */

static bool adc_last_ok = false;

static int16_t raw16_to_mv(int16_t raw)
{
    int32_t mv;

    /* With SD24ALGN set, SD24BMEMHn contains the signed upper 16 bits.
     * Full scale is +/-32768 counts for +/-Vref at gain = 1.
     */
    mv = (int32_t)raw * (int32_t)SD24_VREF_MV;
    mv /= SD24_FULL_SCALE_16;

    return (int16_t)mv;
}

static int16_t read_raw16_ch0(void)
{
    return (int16_t)SD24BMEMH0;
}

static int16_t read_raw16_ch1(void)
{
    return (int16_t)SD24BMEMH1;
}

static int16_t read_raw16_ch2(void)
{
    return (int16_t)SD24BMEMH2;
}

bool sd24_3ch_last_ok(void)
{
    return adc_last_ok;
}

void sd24_3ch_init(void)
{
    /* SMCLK is 16 MHz. Divide before feeding the SD24_B modulator path. */
    SD24BCTL0 = SD24SSEL__SMCLK | SD24PDIV_7 | SD24REFS;   /* /8, internal SD24 ref */
    SD24BCTL1 = 0;

    /* Same channel grouping style as the TI 3-channel example:
     * converters 0/1/2 in group 0. SD24ALGN makes SD24BMEMHn directly usable
     * as a signed 16-bit result. SD24DF_1 keeps two's-complement format.
     */
    SD24BCCTL0 = SD24ALGN | SD24DF_1 | SD24SCS_4;
    SD24BCCTL1 = SD24ALGN | SD24DF_1 | SD24SCS_4;
    SD24BCCTL2 = SD24ALGN | SD24DF_1 | SD24SCS_4;

    /* Default input pair is SDxP0 - SDxN0. Do not use SD24INCH_0; this
     * device header does not define it, and P0/N0 is already selection 0.
     */
    SD24BINCTL0 = SD24GAIN_1;
    SD24BINCTL1 = SD24GAIN_1;
    SD24BINCTL2 = SD24GAIN_1;

    SD24BOSR0 = OSR__1024;
    SD24BOSR1 = OSR__1024;
    SD24BOSR2 = OSR__1024;

    SD24BPRE0 = 0;
    SD24BPRE1 = 0;
    SD24BPRE2 = 0;

    SD24BIE = 0;
    SD24BIFG = 0;

    delay_ms(10);
}

adc24_sample_t sd24_3ch_read(void)
{
    adc24_sample_t s;
    uint32_t timeout = SD24_TIMEOUT_COUNT;
    uint16_t flags;
    int16_t raw0;
    int16_t raw1;
    int16_t raw2;

    s.ch0_voutwe_gnd = 0;
    s.ch1_vcc1v25_vouttia = 0;
    s.ch2_gnd_voutre = 0;
    s.ch0_mv = 0;
    s.ch1_mv = 0;
    s.ch2_mv = 0;
    adc_last_ok = false;

    /* Ensure group is stopped, clear flags, then start group 0. */
    SD24BCTL1 &= (uint16_t)~SD24GRP0SC;
    SD24BIFG &= (uint16_t)~(SD24IFG0 | SD24IFG1 | SD24IFG2 |
                            SD24OVIFG0 | SD24OVIFG1 | SD24OVIFG2);
    SD24BCTL1 |= SD24GRP0SC;

    /* Wait until all three converters have a settled result. */
    do {
        flags = (uint16_t)(SD24BIFG & (SD24IFG0 | SD24IFG1 | SD24IFG2));
        timeout--;
    } while ((flags != (SD24IFG0 | SD24IFG1 | SD24IFG2)) && (timeout != 0UL));

    SD24BCTL1 &= (uint16_t)~SD24GRP0SC;

    if (timeout == 0UL) {
        return s;
    }

    raw0 = read_raw16_ch0();
    raw1 = read_raw16_ch1();
    raw2 = read_raw16_ch2();

    /* Keep raw fields compatible with the existing struct/CSV output. */
    s.ch0_voutwe_gnd      = (int32_t)raw0;
    s.ch1_vcc1v25_vouttia = (int32_t)raw1;
    s.ch2_gnd_voutre      = (int32_t)raw2;

    s.ch0_mv = raw16_to_mv(raw0);
    s.ch1_mv = raw16_to_mv(raw1);
    s.ch2_mv = raw16_to_mv(raw2);

    adc_last_ok = true;
    return s;
}
