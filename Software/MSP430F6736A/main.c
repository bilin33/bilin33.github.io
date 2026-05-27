#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"
#include "i2c0.h"
#include "uart1.h"
#include "dac80502_ll.h"
#include "sd24_3ch.h"
#include "oled_ssd1306_ll.h"
#include "bme280_port.h"

#define DAC_MIN_MV       0
#define DAC_MAX_MV       2500
#define DAC_STEP_MV      100
#define DAC_DEFAULT_MV   100
#define LOOP_PERIOD_MS   100U
#define OLED_PERIOD_MS   1000U
#define BME_RETRY_MS     5000U

/* PC GUI binary protocol: AA 55 | CMD | LEN_L LEN_H | PAYLOAD | CRC16_L CRC16_H */
#define FRAME_HEAD1      0xAAu
#define FRAME_HEAD2      0x55u

#define CMD_PING         0x01u
#define CMD_SET_DAC      0x10u
#define CMD_READ_ADC     0x11u
#define CMD_START_CA     0x20u
#define CMD_START_CV     0x21u
#define CMD_START_EIS    0x22u
#define CMD_STOP         0x23u

#define CMD_DATA         0x30u
#define CMD_ACK          0x7Fu
#define CMD_ERROR        0x7Eu

/* Default current conversion assumes Rf = 1 Mohm:
 * 1 mV across the TIA output relative to TIABIAS corresponds to 1 nA = 1000 pA.
 * Change CURRENT_PA_PER_MV when you change the selected TIA feedback resistor.
 */
#define CURRENT_PA_PER_MV 1000L


static uint16_t dac_mv = DAC_DEFAULT_MV;
static bool ac_mode = false;
static bool ac_phase_high = true;
static struct bme280_dev bme;
static bool gui_protocol_active = false;
static bool stream_enabled = false;
static uint32_t g_time_ms = 0UL;

/* Small formatting helpers: compatible with CCS --printf_support=minimal because
 * they do not use snprintf(), %f, %lu, %ld, or length modifiers.
 */
static void str_append(char *dst, unsigned dst_len, const char *src)
{
    unsigned i = 0;
    unsigned j = 0;
    if (dst_len == 0) return;
    while ((i + 1U) < dst_len && dst[i] != '\0') i++;
    while ((i + 1U) < dst_len && src[j] != '\0') dst[i++] = src[j++];
    dst[i] = '\0';
}

static void u32_append(char *dst, unsigned dst_len, uint32_t v)
{
    char tmp[11];
    unsigned i = 0;
    if (v == 0UL) {
        str_append(dst, dst_len, "0");
        return;
    }
    while (v != 0UL && i < sizeof(tmp)) {
        tmp[i++] = (char)('0' + (v % 10UL));
        v /= 10UL;
    }
    while (i > 0U) {
        char s[2];
        s[0] = tmp[--i];
        s[1] = '\0';
        str_append(dst, dst_len, s);
    }
}

static void i32_append(char *dst, unsigned dst_len, int32_t v)
{
    if (v < 0L) {
        str_append(dst, dst_len, "-");
        u32_append(dst, dst_len, (uint32_t)(-v));
    } else {
        u32_append(dst, dst_len, (uint32_t)v);
    }
}

static void u32_append_2digits(char *dst, unsigned dst_len, uint32_t v)
{
    char s[3];
    v %= 100UL;
    s[0] = (char)('0' + (v / 10UL));
    s[1] = (char)('0' + (v % 10UL));
    s[2] = '\0';
    str_append(dst, dst_len, s);
}

static void u32_append_3digits(char *dst, unsigned dst_len, uint32_t v)
{
    char s[4];
    v %= 1000UL;
    s[0] = (char)('0' + (v / 100UL));
    s[1] = (char)('0' + ((v / 10UL) % 10UL));
    s[2] = (char)('0' + (v % 10UL));
    s[3] = '\0';
    str_append(dst, dst_len, s);
}

static void apply_keys(void)
{
    static uint8_t old = (KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);
    static uint8_t stable = (KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);
    uint8_t now;
    uint8_t changed_to_pressed;

    now = P4IN & (KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);

    /* Simple debounce: only accept a change after 25 ms and one re-read. */
    if (now != stable) {
        delay_ms(25);
        stable = P4IN & (KEY_PLUS_BIT | KEY_MINUS_BIT | KEY_SW_BIT);
        now = stable;
    }

    changed_to_pressed = (uint8_t)(old & (uint8_t)(~now));
    old = now;

    if (changed_to_pressed & KEY_PLUS_BIT) {
        if (dac_mv + DAC_STEP_MV <= DAC_MAX_MV) dac_mv += DAC_STEP_MV;
    }

    if (changed_to_pressed & KEY_MINUS_BIT) {
        if (dac_mv >= DAC_STEP_MV) dac_mv -= DAC_STEP_MV;
    }

    if (changed_to_pressed & KEY_SW_BIT) {
        ac_mode = !ac_mode;
    }
}

static void update_dac_output(void)
{
    if (!ac_mode) {
        dac80502_set_voltage_mv(dac_mv);
    } else {
        dac80502_set_voltage_mv(ac_phase_high ? dac_mv : 0);
        ac_phase_high = !ac_phase_high;      /* square-wave AC-like output */
    }
}

static void format_bme(char *dst, unsigned len, const struct bme280_data *d)
{
    dst[0] = '\0';
#ifdef BME280_DOUBLE_ENABLE
    /* Do not build bme280.c with BME280_DOUBLE_ENABLE when using
     * --printf_support=minimal. Integer BME280 mode is recommended for MSP430.
     */
    str_append(dst, len, "BME280_DOUBLE_OFF");
#else
    /* Bosch integer mode: temperature 0.01 C, pressure Pa, humidity 1/1024 %RH */
    int32_t temp = d->temperature;
    uint32_t temp_abs;
    str_append(dst, len, "T=");
    if (temp < 0L) {
        str_append(dst, len, "-");
        temp_abs = (uint32_t)(-temp);
    } else {
        temp_abs = (uint32_t)temp;
    }
    u32_append(dst, len, temp_abs / 100UL);
    str_append(dst, len, ".");
    u32_append_2digits(dst, len, temp_abs % 100UL);
    str_append(dst, len, "C P=");
    u32_append(dst, len, d->pressure);
    str_append(dst, len, "Pa H=");
    u32_append(dst, len, d->humidity / 1024UL);
    str_append(dst, len, ".");
    u32_append_3digits(dst, len, ((d->humidity % 1024UL) * 1000UL) / 1024UL);
    str_append(dst, len, "%");
#endif
}

static void format_csv_line(char *dst, unsigned len, uint32_t sample, const char *bme_txt, const adc24_sample_t *adc)
{
    dst[0] = '\0';
    u32_append(dst, len, sample);
    str_append(dst, len, ",");
    u32_append(dst, len, dac_mv);
    str_append(dst, len, ",");
    str_append(dst, len, ac_mode ? "AC" : "DC");
    str_append(dst, len, ",");
    str_append(dst, len, bme_txt);
    str_append(dst, len, ",");
    i32_append(dst, len, adc->ch0_voutwe_gnd);
    str_append(dst, len, ",");
    i32_append(dst, len, adc->ch0_mv);
    str_append(dst, len, ",");
    i32_append(dst, len, adc->ch1_vcc1v25_vouttia);
    str_append(dst, len, ",");
    i32_append(dst, len, adc->ch1_mv);
    str_append(dst, len, ",");
    i32_append(dst, len, adc->ch2_gnd_voutre);
    str_append(dst, len, ",");
    i32_append(dst, len, adc->ch2_mv);
    str_append(dst, len, "\r\n");
}


static uint16_t protocol_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
            else crc >>= 1;
        }
    }
    return crc;
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void protocol_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t frame[80];
    uint16_t idx = 0;
    uint16_t crc;
    uint16_t i;

    if (len > 64U) return;

    frame[idx++] = FRAME_HEAD1;
    frame[idx++] = FRAME_HEAD2;
    frame[idx++] = cmd;
    frame[idx++] = (uint8_t)(len & 0xFFu);
    frame[idx++] = (uint8_t)(len >> 8);

    for (i = 0; i < len; i++) frame[idx++] = payload[i];

    crc = protocol_crc16(&frame[2], (uint16_t)(idx - 2U));
    frame[idx++] = (uint8_t)(crc & 0xFFu);
    frame[idx++] = (uint8_t)(crc >> 8);

    uart1_send(frame, idx);
}

static void protocol_send_ack(uint8_t ack_cmd)
{
    uint8_t p[1];
    p[0] = ack_cmd;
    protocol_send_frame(CMD_ACK, p, 1);
}

static void protocol_send_error(uint8_t err)
{
    uint8_t p[1];
    p[0] = err;
    protocol_send_frame(CMD_ERROR, p, 1);
}

static void protocol_send_adc_data(const adc24_sample_t *adc)
{
    uint8_t p[20];
    int32_t e_set_uv;
    int32_t e_meas_uv;
    int32_t current_pa;
    int32_t raw;

    /* Existing sd24_3ch.c naming:
     *   ch0_mv = WE - GND
     *   ch1_mv = TIABIAS - TIAOUT, depending on your board wiring comment
     *   ch2_mv = GND - RE
     * Therefore WE - RE ~= ch0_mv + ch2_mv.
     * Change sign here if your board defines RE-WE instead.
     */
    e_set_uv = (int32_t)dac_mv * 1000L;
    e_meas_uv = ((int32_t)adc->ch0_mv + (int32_t)adc->ch2_mv) * 1000L;
    current_pa = (int32_t)adc->ch1_mv * CURRENT_PA_PER_MV;
    raw = adc->ch1_vcc1v25_vouttia;

    put_u32_le(&p[0], g_time_ms);
    put_u32_le(&p[4], (uint32_t)e_set_uv);
    put_u32_le(&p[8], (uint32_t)e_meas_uv);
    put_u32_le(&p[12], (uint32_t)current_pa);
    put_u32_le(&p[16], (uint32_t)raw);

    protocol_send_frame(CMD_DATA, p, 20);
}

static void protocol_handle_command(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    gui_protocol_active = true;

    switch (cmd)
    {
    case CMD_PING:
        protocol_send_ack(CMD_PING);
        break;

    case CMD_SET_DAC:
        if (len >= 3U) {
            uint8_t ch;
            uint16_t code;
            uint32_t mv32;
            ch = payload[0];
            code = get_u16_le(&payload[1]);
            mv32 = ((uint32_t)code * 2500UL + 32767UL) / 65535UL;
            if (mv32 > DAC_MAX_MV) mv32 = DAC_MAX_MV;
            dac_mv = (uint16_t)mv32;
            if (ch == 0U) (void)dac80502_set_code_a(code);
            else (void)dac80502_set_code_b(code);
            protocol_send_ack(CMD_SET_DAC);
        } else {
            protocol_send_error(CMD_SET_DAC);
        }
        break;

    case CMD_READ_ADC:
    {
        adc24_sample_t s_adc;
        s_adc = sd24_3ch_read();
        protocol_send_adc_data(&s_adc);
        break;
    }

    case CMD_START_CA:
        stream_enabled = true;
        protocol_send_ack(CMD_START_CA);
        break;

    case CMD_START_CV:
    case CMD_START_EIS:
        /* Placeholders for now: GUI receives ACK, but full CV/EIS state machines
         * still need to be implemented after Manual + CA are verified.
         */
        stream_enabled = true;
        protocol_send_ack(cmd);
        break;

    case CMD_STOP:
        stream_enabled = false;
        protocol_send_ack(CMD_STOP);
        break;

    default:
        protocol_send_error(cmd);
        break;
    }
}

static void process_uart_protocol(void)
{
    static uint8_t buf[80];
    static uint16_t pos = 0;

    while (uart1_available()) {
        uint8_t b;
        b = uart1_read_byte();

        if (pos == 0U) {
            if (b != FRAME_HEAD1) continue;
            buf[pos++] = b;
            continue;
        }

        if (pos == 1U) {
            if (b != FRAME_HEAD2) {
                pos = 0;
                continue;
            }
            buf[pos++] = b;
            continue;
        }

        if (pos < sizeof(buf)) {
            buf[pos++] = b;
        } else {
            pos = 0;
            continue;
        }

        if (pos >= 5U) {
            uint16_t len;
            uint16_t total;
            len = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
            total = (uint16_t)(2U + 1U + 2U + len + 2U);

            if (len > 64U || total > sizeof(buf)) {
                pos = 0;
                continue;
            }

            if (pos >= total) {
                uint16_t rx_crc;
                uint16_t calc_crc;
                rx_crc = (uint16_t)buf[total - 2U] | ((uint16_t)buf[total - 1U] << 8);
                calc_crc = protocol_crc16(&buf[2], (uint16_t)(total - 4U));
                if (rx_crc == calc_crc) {
                    protocol_handle_command(buf[2], &buf[5], len);
                }
                pos = 0;
            }
        }
    }
}

static void format_dac_display(char *dst, unsigned len)
{
    dst[0] = '\0';
    str_append(dst, len, "DAC:");
    u32_append(dst, len, dac_mv / 1000U);
    str_append(dst, len, ".");
    u32_append(dst, len, (dac_mv % 1000U) / 100U);
    str_append(dst, len, "V");
}

static void append_signed_mv_as_v(char *dst, unsigned len, int16_t mv)
{
    uint16_t abs_mv;
    if (mv < 0) {
        str_append(dst, len, "-");
        abs_mv = (uint16_t)(-mv);
    } else {
        abs_mv = (uint16_t)mv;
    }
    u32_append(dst, len, abs_mv / 1000U);
    str_append(dst, len, ".");
    u32_append_3digits(dst, len, abs_mv % 1000U);
    str_append(dst, len, "V");
}

static void format_adc_display(char *dst, unsigned len, const char *name, int16_t mv)
{
    dst[0] = '\0';
    str_append(dst, len, name);
    str_append(dst, len, ":");
    append_signed_mv_as_v(dst, len, mv);
}

static void format_dac_status(char *dst, unsigned len)
{
    dst[0] = '\0';
    str_append(dst, len, dac80502_last_ok() ? "DAC:OK " : "DAC:ERR ");
    u32_append(dst, len, dac_mv / 1000U);
    str_append(dst, len, ".");
    u32_append(dst, len, (dac_mv % 1000U) / 100U);
    str_append(dst, len, "V C");
    u32_append(dst, len, dac80502_last_code());
}


static void format_bme_status(char *dst, unsigned len, bool bme_ok)
{
    dst[0] = '\0';
    if (bme_ok) {
        str_append(dst, len, "BME:OK 0x");
        str_append(dst, len, "76");
    } else {
        str_append(dst, len, "BME:ERR 0x76");
    }
}

int main(void)
{
    struct bme280_data bme_data;
    adc24_sample_t adc;
    char bme_txt[64];
    char line[192];
    uint32_t sample = 0;
    uint32_t ms_count = 0;
    uint16_t oled_timer = 0;
    uint16_t bme_retry_timer = 0;
    bool bme_ok;

    WDTCTL = WDTPW | WDTHOLD;
    clock_init_16mhz();
    gpio_init();
    i2c0_init_100khz();
    uart1_init_115200();
    __bis_SR_register(GIE);
    sd24_3ch_init();

    uart1_puts("MSP430F6736A sensor/DAC UART start\r\n");
    oled_init();
    oled_print(0, 0, "MSP430F6736A");
    oled_print(0, 8, "Init...");
    oled_update();

    dac80502_init();
    bme_ok = bme280_port_init(&bme);

    adc.ch0_voutwe_gnd = 0;
    adc.ch1_vcc1v25_vouttia = 0;
    adc.ch2_gnd_voutre = 0;
    adc.ch0_mv = 0;
    adc.ch1_mv = 0;
    adc.ch2_mv = 0;
    strcpy(bme_txt, "BME280_WAIT");

    while (1) {
        apply_keys();
        process_uart_protocol();
        update_dac_output();

        adc = sd24_3ch_read();

        if (bme_ok && bme280_port_read(&bme, &bme_data)) {
            format_bme(bme_txt, sizeof(bme_txt), &bme_data);
        } else {
            strcpy(bme_txt, "BME280_ERR");
            bme_ok = false;
        }

        if (!bme_ok) {
            bme_retry_timer += LOOP_PERIOD_MS;
            if (bme_retry_timer >= BME_RETRY_MS) {
                bme_retry_timer = 0;
                bme_ok = bme280_port_init(&bme);     /* retry fixed 0x76 only */
            }
        }

        if (stream_enabled) {
            protocol_send_adc_data(&adc);
        }

        format_csv_line(line, sizeof(line), sample++, sd24_3ch_last_ok() ? bme_txt : "ADC_TIMEOUT", &adc);
        if (!gui_protocol_active) {
            uart1_puts(line);
        }
        /* OLED is updated by a 1-second software timer. This prevents the
         * full SSD1306 refresh from fighting BME280/DAC I2C traffic and stops
         * the visible screen flashing/shaking caused by fast clear+redraw.
         */
        oled_timer += LOOP_PERIOD_MS;
        ms_count += LOOP_PERIOD_MS;
        g_time_ms += LOOP_PERIOD_MS;
        if (oled_timer >= OLED_PERIOD_MS) {
            oled_timer = 0;
            oled_clear();
            format_bme_status(line, sizeof(line), bme_ok);
            oled_print(0, 0, line);
            oled_print(0, 8, ac_mode ? "Mode: AC" : "Mode: DC");
            format_dac_status(line, sizeof(line));
            oled_print(0, 16, line);
            strcpy(line, "Scan:");
            u32_append(line, sizeof(line), ms_count / 1000UL);
            str_append(line, sizeof(line), "s");
            oled_print(72, 16, line);
            oled_print(0, 24, sd24_3ch_last_ok() ? bme_txt : "ADC:TIMEOUT");
            format_adc_display(line, sizeof(line), "A0", adc.ch0_mv); oled_print(0, 40, line);
            format_adc_display(line, sizeof(line), "A1", adc.ch1_mv); oled_print(0, 48, line);
            format_adc_display(line, sizeof(line), "A2", adc.ch2_mv); oled_print(0, 56, line);
            oled_update();
        }

        delay_ms(LOOP_PERIOD_MS);
    }
}
