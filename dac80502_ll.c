#include "dac80502_ll.h"
#include "i2c0.h"
#include "platform.h"

/* DAC80502 robust MSP430 driver.
 * Uses the same 7-bit I2C address and 3-byte register write format as the
 * working reference file:
 *   START + 0x48(W) + register + data_MSB + data_LSB + STOP
 *
 * Important changes from the previous project:
 *   1) Explicitly powers up internal reference/DAC.
 *   2) Sets gain/reference divider for a 0..2.5 V full-scale range.
 *   3) Writes BOTH DAC-A and DAC-B, so output appears even if your board uses
 *      VOUTB instead of VOUTA.
 */
#define DAC80502_REG_SYNC       0x02u
#define DAC80502_REG_CONFIG     0x03u
#define DAC80502_REG_GAIN       0x04u
#define DAC80502_REG_TRIGGER    0x05u
#define DAC80502_REG_DAC_A      0x08u
#define DAC80502_REG_DAC_B      0x09u

#define DAC80502_FS_MV          2500UL

static bool dac_ok = false;
static uint16_t last_code = 0u;

static bool dac_write16(uint8_t reg, uint16_t value)
{
    uint8_t d[2];
    bool ok;

    d[0] = (uint8_t)(value >> 8);
    d[1] = (uint8_t)(value & 0xFFu);

    ok = i2c0_write_reg(DAC80502_ADDR, reg, d, 2);
    dac_ok = ok;
    delay_us(50);              /* give DAC/I2C bus a short recovery time */
    return ok;
}

bool dac80502_last_ok(void)
{
    return dac_ok;
}

uint16_t dac80502_last_code(void)
{
    return last_code;
}

bool dac80502_set_code_a(uint16_t code)
{
    last_code = code;
    return dac_write16(DAC80502_REG_DAC_A, code);
}

bool dac80502_set_code_b(uint16_t code)
{
    last_code = code;
    return dac_write16(DAC80502_REG_DAC_B, code);
}

bool dac80502_set_voltage_mv(uint16_t mv)
{
    uint32_t code32;
    uint16_t code;
    bool ok_a, ok_b;

    if (mv > DAC80502_FS_MV) mv = DAC80502_FS_MV;
    code32 = ((uint32_t)mv * 65535UL + (DAC80502_FS_MV / 2UL)) / DAC80502_FS_MV;
    code = (uint16_t)code32;
    last_code = code;

    /* Write both channels to make debugging easier and avoid A/B wiring mixup. */
    ok_a = dac_write16(DAC80502_REG_DAC_A, code);
    ok_b = dac_write16(DAC80502_REG_DAC_B, code);
    dac_ok = (ok_a && ok_b);
    return dac_ok;
}

void dac80502_init(void)
{
    bool ok = true;

    delay_ms(10);

    /* CONFIG 0x0000: internal reference powered, DAC outputs powered. */
    ok &= dac_write16(DAC80502_REG_CONFIG, 0x0000u);
    delay_ms(2);

    /* SYNC 0x0000: asynchronous update, DAC output updates immediately after
     * writing DAC-A/DAC-B data registers.
     */
    ok &= dac_write16(DAC80502_REG_SYNC, 0x0000u);

    /* GAIN 0x0101: reference divide-by-2 and buffer gain x2. With the 2.5 V
     * internal reference this gives a 0..2.5 V full-scale output range.
     */
    ok &= dac_write16(DAC80502_REG_GAIN, 0x0101u);

    /* TRIGGER 0x0010 is harmless in async mode and forces a software LDAC in
     * case the device was previously left in sync mode by older code.
     */
    ok &= dac_write16(DAC80502_REG_TRIGGER, 0x0010u);
    delay_ms(2);

    /* Default project requirement: 1.2 V DC at startup. */
    ok &= dac80502_set_voltage_mv(1200u);
    dac_ok = ok;
}
