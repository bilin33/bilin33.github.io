#ifndef SD24_3CH_H_
#define SD24_3CH_H_
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t ch0_voutwe_gnd;
    int32_t ch1_vcc1v25_vouttia;
    int32_t ch2_gnd_voutre;
    int16_t ch0_mv;
    int16_t ch1_mv;
    int16_t ch2_mv;
} adc24_sample_t;

void sd24_3ch_init(void);
adc24_sample_t sd24_3ch_read(void);
bool sd24_3ch_last_ok(void);

#endif
