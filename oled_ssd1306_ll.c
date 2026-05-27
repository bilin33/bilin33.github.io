#include <stdint.h>
#include <string.h>
#include "i2c0.h"
#include "font6x8.h"
#include "oled_ssd1306_ll.h"

#define SSD1306_ADDR        0x3C
#define SSD1306_CTRL_CMD    0x00
#define SSD1306_CTRL_DATA   0x40
static uint8_t fb[1024];

static void cmd(uint8_t c) { uint8_t b[2] = {SSD1306_CTRL_CMD, c}; i2c0_write(SSD1306_ADDR, b, 2); }
static void data_chunk(const uint8_t *p, uint8_t n)
{
    uint8_t b[17];
    uint8_t i;
    b[0] = SSD1306_CTRL_DATA;
    for (i = 0; i < n; i++) b[i + 1] = p[i];
    i2c0_write(SSD1306_ADDR, b, n + 1);
}

void oled_init(void)
{
    static const uint8_t init[] = {0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,0x8D,0x14,
                                  0x20,0x00,0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,
                                  0xDB,0x40,0xA4,0xA6,0xAF};
    uint8_t i;
    for (i = 0; i < sizeof(init); i++) cmd(init[i]);
    oled_clear();
    oled_update();
}

void oled_clear(void) { memset(fb, 0, sizeof(fb)); }

static void pixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= 128 || y >= 64) return;
    if (on) fb[x + (y / 8) * 128] |=  (1u << (y & 7));
    else    fb[x + (y / 8) * 128] &= ~(1u << (y & 7));
}

void oled_print(uint8_t x, uint8_t y, const char *s)
{
    while (*s) {
        uint8_t c = (uint8_t)*s++;
        uint8_t col, row;
        if (c < 0x20 || c > 0x7F) c = '?';
        for (col = 0; col < 6; col++) {
            uint8_t line = font6x8_ascii[c][col];
            for (row = 0; row < 8; row++) pixel(x + col, y + row, (line >> row) & 1);
        }
        x += 6;
        if (x > 122) { x = 0; y += 8; }
    }
}

void oled_update(void)
{
    uint16_t i;
    cmd(0x21); cmd(0); cmd(127);
    cmd(0x22); cmd(0); cmd(7);
    for (i = 0; i < 1024; i += 16) data_chunk(&fb[i], 16);
}
