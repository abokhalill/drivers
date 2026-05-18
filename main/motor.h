#pragma once

#include <stdint.h>

void motor_init(void);
void servo_set_us(uint8_t ch, uint16_t pulse_us);  // ch, us
void drive_set_throttle(float throttle);            // -1..+1
void steering_set(float steer);                     // -1..+1
