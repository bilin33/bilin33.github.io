#ifndef OLED_SSD1306_LL_H_
#define OLED_SSD1306_LL_H_
void oled_init(void);
void oled_clear(void);
void oled_print(uint8_t x, uint8_t y, const char *s);
void oled_update(void);
#endif
