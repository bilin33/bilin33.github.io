#ifndef BME280_PORT_H_
#define BME280_PORT_H_
#include <stdbool.h>
#include <stdint.h>
/* MSP430 build: force Bosch BME280 integer output.
 * The CC2640 project can use BME280_DOUBLE_ENABLE because it has full printf/float support,
 * but this MSP430 project uses minimal printf/manual formatting.
 */
#ifndef BME280_32BIT_ENABLE
#define BME280_32BIT_ENABLE
#endif
#ifdef BME280_DOUBLE_ENABLE
#undef BME280_DOUBLE_ENABLE
#endif
#include "bme280.h"
bool bme280_port_init(struct bme280_dev *dev);
bool bme280_port_read(struct bme280_dev *dev, struct bme280_data *data);
uint8_t bme280_port_addr(void);
uint8_t bme280_port_id(void);
#endif
