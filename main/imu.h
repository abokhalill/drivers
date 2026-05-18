#pragma once

#include <stdbool.h>

void imu_init(void);
bool imu_read_raw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void imu_compute_angles(float ax, float ay, float az, float *roll, float *pitch);

// core 1
void task_imu(void *arg);
