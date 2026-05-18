#pragma once

#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef enum {
    SUSPENSION_MODE_PASSIVE = 0,
    SUSPENSION_MODE_ACTIVE  = 1,
} suspension_mode_t;

typedef struct {
    // atomic; lock-free hot path
    _Atomic float drive_cmd;        // -1.0 to +1.0
    _Atomic float steer_cmd;        // -1.0 to +1.0

    // atomic; imu->pid
    _Atomic float roll_deg;
    _Atomic float pitch_deg;
    _Atomic float accel_z_g;

    // atomic; bt callback
    _Atomic suspension_mode_t mode;

    // mutex: telem reads
    uint16_t servo_duty[4];         // FL, FR, RL, RR

    // mutex: ring buffer
    float log_roll[512];
    float log_pitch[512];
    uint32_t log_index;

    SemaphoreHandle_t mutex;        // servo_duty + log only
} car_state_t;

extern car_state_t g_car;

void shared_state_init(void);
