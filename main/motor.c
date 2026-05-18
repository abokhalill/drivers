#include "motor.h"
#include "config.h"
#include "pca9685.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "motor";

// ch0=rpwm(fwd), ch1=lpwm(rev)
static void bts_set(uint32_t rpwm_duty, uint32_t lpwm_duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, rpwm_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, lpwm_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void motor_init(void)
{
    gpio_reset_pin(BTS_R_EN_GPIO);
    gpio_reset_pin(BTS_L_EN_GPIO);
    gpio_set_direction(BTS_R_EN_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(BTS_L_EN_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(BTS_R_EN_GPIO, 1);
    gpio_set_level(BTS_L_EN_GPIO, 1);

    ESP_LOGI(TAG, "enable pins: R_EN(GPIO%d)=%d L_EN(GPIO%d)=%d",
             BTS_R_EN_GPIO, gpio_get_level(BTS_R_EN_GPIO),
             BTS_L_EN_GPIO, gpio_get_level(BTS_L_EN_GPIO));

    // reset dac pins before ledc
    gpio_reset_pin(BTS_RPWM_GPIO);
    gpio_reset_pin(BTS_LPWM_GPIO);

    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = BTS_PWM_RES,
        .freq_hz          = BTS_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // fwd
    ledc_channel_config_t rpwm_ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = BTS_RPWM_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&rpwm_ch));

    // rev
    ledc_channel_config_t lpwm_ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = BTS_LPWM_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&lpwm_ch));

    ESP_LOGI(TAG, "bts7960 ready: rpwm=GPIO%d lpwm=GPIO%d r_en=GPIO%d l_en=GPIO%d",
             BTS_RPWM_GPIO, BTS_LPWM_GPIO, BTS_R_EN_GPIO, BTS_L_EN_GPIO);

    // pca9685; shared i2c
    pca9685_init();
}

void servo_set_us(uint8_t ch, uint16_t pulse_us)
{
    pca9685_set_pulse(ch, pulse_us);
}

void drive_set_throttle(float throttle)
{
    if (throttle >  1.0f) throttle =  1.0f;
    if (throttle < -1.0f) throttle = -1.0f;

    if (throttle > 0.0f) {
        // fwd
        uint32_t duty = (uint32_t)(throttle * BTS_DUTY_MAX);
        bts_set(duty, 0);
    } else if (throttle < 0.0f) {
        // rev
        uint32_t duty = (uint32_t)(-throttle * BTS_DUTY_MAX);
        bts_set(0, duty);
    } else {
        // stop
        bts_set(0, 0);
    }
}

void steering_set(float steer)
{
    if (steer >  1.0f) steer =  1.0f;
    if (steer < -1.0f) steer = -1.0f;
    float fraction = (steer + 1.0f) / 2.0f;
    uint16_t us = SERVO_US_MIN + (uint16_t)(fraction * (SERVO_US_MAX - SERVO_US_MIN));
    pca9685_set_pulse(PCA9685_CH_STEER, us);
}
