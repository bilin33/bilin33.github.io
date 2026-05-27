#include <stdint.h>
#include <stdbool.h>
#include "bme280_port.h"
#include "i2c0.h"
#include "platform.h"

/* This BME280 code is intentionally copied from the single-file version that
 * works on your hardware. It does not use the Bosch callback driver path.
 */
#define BME280_ADDR 0x76u

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;
} bme280_calib_local_t;

static bme280_calib_local_t g_bme;
static uint8_t g_bme_id = 0;

static uint16_t u16_le(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static int16_t  s16_le(const uint8_t *p) { return (int16_t)u16_le(p); }

static bool bme_write_u8(uint8_t reg, uint8_t v)
{
    return i2c0_write_reg(BME280_ADDR, reg, &v, 1);
}

static bool bme_read(uint8_t reg, uint8_t *dst, uint8_t len)
{
    return i2c0_read_reg(BME280_ADDR, reg, dst, len);
}

static bool bme280_read_calibration_local(void)
{
    uint8_t buf1[26];
    uint8_t buf2[7];

    if (!bme_read(0x88, buf1, 26)) return false;
    if (!bme_read(0xE1, buf2, 7)) return false;

    g_bme.dig_T1 = u16_le(&buf1[0]);
    g_bme.dig_T2 = s16_le(&buf1[2]);
    g_bme.dig_T3 = s16_le(&buf1[4]);

    g_bme.dig_P1 = u16_le(&buf1[6]);
    g_bme.dig_P2 = s16_le(&buf1[8]);
    g_bme.dig_P3 = s16_le(&buf1[10]);
    g_bme.dig_P4 = s16_le(&buf1[12]);
    g_bme.dig_P5 = s16_le(&buf1[14]);
    g_bme.dig_P6 = s16_le(&buf1[16]);
    g_bme.dig_P7 = s16_le(&buf1[18]);
    g_bme.dig_P8 = s16_le(&buf1[20]);
    g_bme.dig_P9 = s16_le(&buf1[22]);

    g_bme.dig_H1 = buf1[25];
    g_bme.dig_H2 = s16_le(&buf2[0]);
    g_bme.dig_H3 = buf2[2];
    g_bme.dig_H4 = (int16_t)(((int16_t)buf2[3] << 4) | (buf2[4] & 0x0F));
    g_bme.dig_H5 = (int16_t)(((int16_t)buf2[5] << 4) | (buf2[4] >> 4));
    g_bme.dig_H6 = (int8_t)buf2[6];
    g_bme.t_fine = 0;
    return true;
}

static bool bme280_read_raw_local(int32_t *adc_T, int32_t *adc_P, int32_t *adc_H)
{
    uint8_t buf[8];
    if (!bme_read(0xF7, buf, 8)) return false;

    *adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((buf[2] >> 4) & 0x0F);
    *adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((buf[5] >> 4) & 0x0F);
    *adc_H = ((int32_t)buf[6] << 8)  |  (int32_t)buf[7];
    return true;
}

static int32_t bme280_compensate_T_local(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)g_bme.dig_T1 << 1))) * ((int32_t)g_bme.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)g_bme.dig_T1)) * ((adc_T >> 4) - ((int32_t)g_bme.dig_T1))) >> 12) *
            ((int32_t)g_bme.dig_T3)) >> 14;
    g_bme.t_fine = var1 + var2;
    return (g_bme.t_fine * 5 + 128) >> 8; /* 0.01 C */
}

static uint32_t bme280_compensate_P_local(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)g_bme.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)g_bme.dig_P6;
    var2 = var2 + ((var1 * (int64_t)g_bme.dig_P5) << 17);
    var2 = var2 + (((int64_t)g_bme.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)g_bme.dig_P3) >> 8) + ((var1 * (int64_t)g_bme.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)g_bme.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)g_bme.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)g_bme.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)g_bme.dig_P7) << 4);
    return (uint32_t)(p >> 8); /* Pa */
}

static uint32_t bme280_compensate_H_local(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (g_bme.t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)g_bme.dig_H4) << 20) - (((int32_t)g_bme.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)g_bme.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)g_bme.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)g_bme.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)g_bme.dig_H1)) >> 4));
    if (v_x1_u32r < 0) v_x1_u32r = 0;
    if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;
    return (uint32_t)(v_x1_u32r >> 12); /* 1024 * %RH */
}

bool bme280_port_init(struct bme280_dev *dev)
{
    (void)dev;
    g_bme_id = 0;
    (void)bme_read(0xD0, &g_bme_id, 1);

    /* Same working startup sequence: reset, wait, calibration, minimal config. */
    (void)bme_write_u8(0xE0, 0xB6);
    delay_ms(10);
    if (!bme280_read_calibration_local()) return false;
    if (!bme_write_u8(0xF2, 0x01)) return false; /* humidity x1 */
    if (!bme_write_u8(0xF5, 0x00)) return false; /* no filter, 0.5ms standby */
    if (!bme_write_u8(0xF4, (uint8_t)((0x01u << 5) | (0x01u << 2) | 0x03u))) return false;
    return true;
}

bool bme280_port_read(struct bme280_dev *dev, struct bme280_data *data)
{
    int32_t adc_T, adc_P, adc_H;
    (void)dev;
    if (!data) return false;
    if (!bme280_read_raw_local(&adc_T, &adc_P, &adc_H)) return false;
    data->temperature = bme280_compensate_T_local(adc_T);
    data->pressure = bme280_compensate_P_local(adc_P);
    data->humidity = bme280_compensate_H_local(adc_H);
    return true;
}

uint8_t bme280_port_addr(void)
{
    return BME280_ADDR;
}

uint8_t bme280_port_id(void)
{
    return g_bme_id;
}
