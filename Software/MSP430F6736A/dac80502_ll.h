#ifndef DAC80502_LL_H_
#define DAC80502_LL_H_
#include <stdint.h>
#include <stdbool.h>
#define DAC80502_ADDR       0x48
void dac80502_init(void);
bool dac80502_set_voltage_mv(uint16_t mv);
bool dac80502_set_code_a(uint16_t code);
bool dac80502_set_code_b(uint16_t code);
bool dac80502_last_ok(void);
uint16_t dac80502_last_code(void);
#endif
