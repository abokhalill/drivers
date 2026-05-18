#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "pca9685.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "main";

// pca9685 channels
#define SUS_CH_A        0       // sus a
#define SUS_CH_B        1       // sus b
#define STEER_CH        2       // steer

#define SERVO_MID_US    1500
#define SERVO_RANGE_US  1000    // ±1000us
#define SUS_OFFSET_US   317     // manual flick offset
#define SUS_MAX_US      500     // max correction

#define PID_KP          3.0f
#define PID_KI          0.1f
#define PID_KD          0.3f
#define PID_IMAX        100.0f  // anti-windup
#define DEADBAND_DEG    3.0f
#define LPF_ALPHA       0.3f

#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

// mpu6050
#define MPU6050_ADDR        0x68
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

static i2c_master_dev_handle_t mpu_handle;

// bts7960 h-bridge
#define BTS_RPWM_GPIO   25
#define BTS_LPWM_GPIO   26
#define MOTOR_FREQ      20000
#define MOTOR_RES       LEDC_TIMER_10_BIT
#define MOTOR_MAX       1023
#define RPWM_CH         LEDC_CHANNEL_0
#define LPWM_CH         LEDC_CHANNEL_1

#define UART_PORT       UART_NUM_0
#define UART_BUF        256

static volatile bool active_mode = false;  // 0=manual, 1=pid
static volatile bool telem_enabled = false; // enabled by P cmd
static SemaphoreHandle_t i2c_mutex;

// telem state
static volatile float telem_roll = 0.0f;
static volatile float telem_pitch = 0.0f;
static volatile float telem_motor = 0.0f;
static volatile float telem_steer = 0.0f;
static volatile uint16_t telem_us_a = 1500;
static volatile uint16_t telem_us_b = 1500;
static volatile int16_t telem_ax = 0;

static void motor_set(float throttle)
{
    if (throttle >  1.0f) throttle =  1.0f;
    if (throttle < -1.0f) throttle = -1.0f;

    uint32_t duty = (uint32_t)(fabsf(throttle) * MOTOR_MAX);
    static int mot_log = 0;
    if (++mot_log % 100 == 1) {
        ESP_LOGI(TAG, "motor_set: t=%.3f duty=%lu", throttle, (unsigned long)duty);
    }

    if (throttle > 0.01f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, RPWM_CH, duty);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LPWM_CH, 0);
    } else if (throttle < -0.01f) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, RPWM_CH, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LPWM_CH, duty);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, RPWM_CH, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LPWM_CH, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, RPWM_CH);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LPWM_CH);
}

// 100hz, core 1
static void suspension_pid_task(void *arg)
{
    float filtered_roll = 0.0f;
    float roll_offset = 0.0f;
    bool calibrated = false;
    int mpu_fail_cnt = 0;
    int stale_cnt = 0;
    int16_t prev_raw_ax = 0;
    TickType_t last_log_tick = 0;

    // imu settle
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "PID suspension task started");

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        if (!active_mode) {
            filtered_roll = 0.0f;
            calibrated = false;
            continue;
        }

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        uint8_t reg = MPU6050_ACCEL_XOUT_H;
        uint8_t raw[6];
        esp_err_t rd_err = i2c_master_transmit_receive(mpu_handle, &reg, 1, raw, 6, 100);
        xSemaphoreGive(i2c_mutex);
        if (rd_err != ESP_OK) {
            mpu_fail_cnt++;
            if (mpu_fail_cnt % 50 == 1) {
                ESP_LOGW(TAG, "mpu read fail #%d", mpu_fail_cnt);
            }
            if (mpu_fail_cnt >= 20 && mpu_fail_cnt % 20 == 0) {
                ESP_LOGW(TAG, "mpu reset recovery");
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                uint8_t rst[2] = { MPU6050_PWR_MGMT_1, 0x80 };
                i2c_master_transmit(mpu_handle, rst, 2, 100);
                vTaskDelay(pdMS_TO_TICKS(50));
                uint8_t wake[2] = { MPU6050_PWR_MGMT_1, 0x00 };
                i2c_master_transmit(mpu_handle, wake, 2, 100);
                vTaskDelay(pdMS_TO_TICKS(20));
                xSemaphoreGive(i2c_mutex);
            }
            continue;
        }
        mpu_fail_cnt = 0;
        int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
        float ax = raw_ax / 16384.0f;
        float ay = (int16_t)((raw[2] << 8) | raw[3]) / 16384.0f;
        float az = (int16_t)((raw[4] << 8) | raw[5]) / 16384.0f;

        // frozen sensor detection
        if (raw_ax == prev_raw_ax) {
            stale_cnt++;
            if (stale_cnt == 50) {
                ESP_LOGW(TAG, "mpu stale data, resetting (ax=%d)", raw_ax);
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                uint8_t rst[2] = { MPU6050_PWR_MGMT_1, 0x80 };
                i2c_master_transmit(mpu_handle, rst, 2, 100);
                vTaskDelay(pdMS_TO_TICKS(50));
                uint8_t wake[2] = { MPU6050_PWR_MGMT_1, 0x00 };
                i2c_master_transmit(mpu_handle, wake, 2, 100);
                vTaskDelay(pdMS_TO_TICKS(20));
                xSemaphoreGive(i2c_mutex);
                stale_cnt = 0;
                continue;
            }
        } else {
            stale_cnt = 0;
            prev_raw_ax = raw_ax;
        }

        float roll_raw = atan2f(ay, az) * (180.0f / M_PI);

        // calibrate zero
        if (!calibrated) {
            roll_offset = roll_raw;
            calibrated = true;
            ESP_LOGI(TAG, "roll calibrated: offset=%.1f", roll_offset);
        }

        // relative to rest
        float roll = roll_raw - roll_offset;

        // periodic log
        TickType_t now_tick = xTaskGetTickCount();
        if (now_tick - last_log_tick > pdMS_TO_TICKS(2000)) {
            last_log_tick = now_tick;
            ESP_LOGI(TAG, "roll=%.1f (raw=%.1f off=%.1f) ax=%d", roll, roll_raw, roll_offset, raw_ax);
        }

        filtered_roll = filtered_roll + LPF_ALPHA * (roll - filtered_roll);

        // proportional: full extension at 10deg
        float gain = (float)SUS_MAX_US / 10.0f;
        float output = filtered_roll * gain;

        // clamp
        if (output >  (float)SUS_MAX_US) output =  (float)SUS_MAX_US;
        if (output < -(float)SUS_MAX_US) output = -(float)SUS_MAX_US;

        if (fabsf(filtered_roll) < DEADBAND_DEG) output = 0.0f;

        // pitch: both same direction
        uint16_t us_a = SERVO_MID_US + (int16_t)output;
        uint16_t us_b = SERVO_MID_US + (int16_t)output;

        // telem
        telem_roll = filtered_roll;
        telem_pitch = atan2f(-ax, az) * (180.0f / M_PI) - roll_offset;
        telem_us_a = us_a;
        telem_us_b = us_b;
        telem_ax = raw_ax;

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        pca9685_set_pulse(SUS_CH_A, us_a);
        pca9685_set_pulse(SUS_CH_B, us_b);
        xSemaphoreGive(i2c_mutex);
    }
}

static bool sus_up = false;

static void uart_rx_task(void *arg)
{
    char buf[UART_BUF];
    int pos = 0;

    float last_steer = 0.0f;
    float last_motor = 0.0f;
    bool steer_dirty = false;
    bool motor_dirty = false;

    while (1) {
        uint8_t chunk[64];
        int n = uart_read_bytes(UART_PORT, chunk, sizeof(chunk), pdMS_TO_TICKS(2));
        if (n <= 0) {
            if (steer_dirty) {
                xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                pca9685_set_pulse(STEER_CH, SERVO_MID_US + (int16_t)(last_steer * SERVO_RANGE_US));
                xSemaphoreGive(i2c_mutex);
                steer_dirty = false;
            }
            if (motor_dirty) {
                motor_set(last_motor);
                motor_dirty = false;
            }
            continue;
        }

        for (int i = 0; i < n; i++) {
            if (chunk[i] == '\n' || chunk[i] == '\r') {
                if (pos > 0) {
                    buf[pos] = '\0';
                    float v;
                    if (buf[0] == 'B') {
                        active_mode = !active_mode;
                        ESP_LOGI(TAG, "mode: %s", active_mode ? "ACTIVE (PID)" : "MANUAL");
                        if (!active_mode) {
                            xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                            pca9685_set_pulse(SUS_CH_A, SERVO_MID_US);
                            pca9685_set_pulse(SUS_CH_B, SERVO_MID_US);
                            xSemaphoreGive(i2c_mutex);
                        }
                    } else if (buf[0] == 'A' && !active_mode) {
                        sus_up = !sus_up;
                        uint16_t us = sus_up ? SERVO_MID_US + SUS_OFFSET_US : SERVO_MID_US - SUS_OFFSET_US;
                        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
                        pca9685_set_pulse(SUS_CH_A, us);
                        pca9685_set_pulse(SUS_CH_B, us);
                        xSemaphoreGive(i2c_mutex);
                    } else if (buf[0] == 'S' && sscanf(buf + 1, "%f", &v) == 1) {
                        if (v >  1.0f) v =  1.0f;
                        if (v < -1.0f) v = -1.0f;
                        last_steer = v;
                        telem_steer = v;
                        steer_dirty = true;
                    } else if (buf[0] == 'M' && sscanf(buf + 1, "%f", &v) == 1) {
                        if (v >  1.0f) v =  1.0f;
                        if (v < -1.0f) v = -1.0f;
                        last_motor = v;
                        telem_motor = v;
                        motor_dirty = true;
                    }
                    pos = 0;
                }
            } else if (pos < UART_BUF - 1) {
                buf[pos++] = (char)chunk[i];
            }
        }
    }
}

static void motor_init(void)
{
    gpio_reset_pin(BTS_RPWM_GPIO);
    gpio_reset_pin(BTS_LPWM_GPIO);

    ledc_timer_config_t mt = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = MOTOR_RES,
        .freq_hz         = MOTOR_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&mt));

    ledc_channel_config_t rpwm = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = RPWM_CH,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = BTS_RPWM_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&rpwm));

    ledc_channel_config_t lpwm = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LPWM_CH,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = BTS_LPWM_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&lpwm));

    ESP_LOGI(TAG, "motor: RPWM=GPIO%d LPWM=GPIO%d 20kHz", BTS_RPWM_GPIO, BTS_LPWM_GPIO);
}

void app_main(void)
{
    i2c_mutex = xSemaphoreCreateMutex();

    // i2c bus
    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_LOGI(TAG, "i2c bus: SDA=GPIO%d SCL=GPIO%d", I2C_SDA_PIN, I2C_SCL_PIN);

    // mpu6050
    i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &mpu_cfg, &mpu_handle));
    uint8_t wake_cmd[2] = { MPU6050_PWR_MGMT_1, 0x00 };
    esp_err_t mpu_ret = i2c_master_transmit(mpu_handle, wake_cmd, 2, 100);
    if (mpu_ret == ESP_OK) {
        ESP_LOGI(TAG, "mpu6050 awake at 0x%02x", MPU6050_ADDR);
    } else {
        ESP_LOGE(TAG, "mpu6050 wake failed: %s (check wiring)", esp_err_to_name(mpu_ret));
    }

    // pca9685: outputs off until first cmd
    pca9685_init();

    motor_init();

    uart_driver_install(UART_PORT, UART_BUF * 2, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "ready: S=steer M=motor A=sus(manual) B=mode toggle");

    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(suspension_pid_task, "pid_sus", 4096, NULL, 7, NULL, 1);

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
