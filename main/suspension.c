#include "suspension.h"
#include "config.h"
#include "shared_state.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

void pid_init(pid_t *pid, float kp, float ki, float kd, float integral_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->integral_limit = integral_limit;
}

float pid_compute(pid_t *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    // deadband: mg995 slop
    if (fabsf(error) < SERVO_DEADBAND_DEG) {
        pid->prev_error = error;
        return 0.0f;
    }

    pid->integral += error * dt;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

void pid_reset(pid_t *pid)
{
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

static inline float clamp(float v, float limit)
{
    return v > limit ? limit : (v < -limit ? -limit : v);
}

void task_suspension(void *arg)
{
    pid_t roll_pid, pitch_pid;
    pid_init(&roll_pid,  PID_KP_ROLL,  PID_KI_ROLL,  PID_KD_ROLL,  30.0f);
    pid_init(&pitch_pid, PID_KP_PITCH, PID_KI_PITCH, PID_KD_PITCH, 30.0f);

    const float dt = 1.0f / 100.0f;
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();
    bool passive_written = false;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        float roll  = atomic_load(&g_car.roll_deg);
        float pitch = atomic_load(&g_car.pitch_deg);
        suspension_mode_t mode = atomic_load(&g_car.mode);

        static const uint8_t susp_ch[4] = {
            PCA9685_CH_FL, PCA9685_CH_FR, PCA9685_CH_RL, PCA9685_CH_RR
        };

        if (mode == SUSPENSION_MODE_PASSIVE) {
            if (!passive_written) {
                for (int i = 0; i < 4; i++)
                    servo_set_us(susp_ch[i], SERVO_US_NEUTRAL);
                pid_reset(&roll_pid);
                pid_reset(&pitch_pid);
                passive_written = true;
            }
            continue;
        }
        passive_written = false;

        // active mode
        float roll_out  = pid_compute(&roll_pid,  0.0f, roll,  dt);
        float pitch_out = pid_compute(&pitch_pid, 0.0f, pitch, dt);

        float range = (float)SERVO_ACTIVE_RANGE;

        // corner signs: fl(roll+,pitch+) fr(-,+) rl(+,-) rr(-,-)
        float corrections[4] = {
            clamp( roll_out + pitch_out, range),
            clamp(-roll_out + pitch_out, range),
            clamp( roll_out - pitch_out, range),
            clamp(-roll_out - pitch_out, range),
        };

        xSemaphoreTake(g_car.mutex, portMAX_DELAY);
        for (int i = 0; i < 4; i++) {
            uint16_t us = SERVO_US_NEUTRAL + (int16_t)corrections[i];
            g_car.servo_duty[i] = us;
            servo_set_us(susp_ch[i], us);
        }
        xSemaphoreGive(g_car.mutex);
    }
}
