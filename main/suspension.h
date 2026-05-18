#pragma once

typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integral;
    float integral_limit;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit);
float pid_compute(pid_t *pid, float setpoint, float measured, float dt);
void pid_reset(pid_t *pid);

// core 1
void task_suspension(void *arg);
