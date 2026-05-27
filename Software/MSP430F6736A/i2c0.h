#ifndef I2C0_H_
#define I2C0_H_
#include <stdint.h>
#include <stdbool.h>
void i2c0_init_100khz(void);
bool i2c0_write(uint8_t addr, const uint8_t *data, uint16_t len);
bool i2c0_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);
bool i2c0_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);
#endif
