#pragma once

#include <stdint.h>

void pca9685_init(void);
void pca9685_set_pulse(uint8_t ch, uint16_t pulse_us);
void pca9685_set_all_pulse(uint16_t pulse_us);
