#include "control.h"
#include "config.h"
#include "shared_state.h"
#include "motor.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "ctrl";

#define CTRL_UART       UART_NUM_0
#define CTRL_BUF_SIZE   256

void control_init(void)
{
    esp_err_t err = uart_driver_install(CTRL_UART, CTRL_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_LOGI(TAG, "uart_driver_install: %s (0x%x)", esp_err_to_name(err), err);
}

// protocol: "C <steer> <throttle> <mode>\n"
// steer:    -1.0 to +1.0 (left/right)
// throttle: -1.0 to +1.0 (reverse/forward)
// mode:     0=passive, 1=active
static void parse_command(const char *line)
{
    float steer = 0, throttle = 0;
    int mode = 0;

    int n = sscanf(line, "C %f %f %d", &steer, &throttle, &mode);
    if (n == 3) {
        static int cmd_count = 0;
        if ((cmd_count++ % 40) == 0)
            ESP_LOGI(TAG, "cmd #%d: steer=%.2f thr=%.2f mode=%d", cmd_count, steer, throttle, mode);

        atomic_store(&g_car.steer_cmd, steer);
        atomic_store(&g_car.drive_cmd, throttle);
        atomic_store(&g_car.mode, mode ? SUSPENSION_MODE_ACTIVE : SUSPENSION_MODE_PASSIVE);

        steering_set(steer);
        drive_set_throttle(throttle);
        servo_set_us(PCA9685_CH_FL, SERVO_US_MIN + (uint16_t)(((steer + 1.0f) / 2.0f) * (SERVO_US_MAX - SERVO_US_MIN)));
    } else {
        ESP_LOGW(TAG, "parse fail (n=%d): '%s'", n, line);
    }
}

void task_control(void *arg)
{
    char buf[CTRL_BUF_SIZE];
    int pos = 0;

    while (1) {
        uint8_t byte;
        int len = uart_read_bytes(CTRL_UART, &byte, 1, pdMS_TO_TICKS(20));
        if (len <= 0) continue;

        if (byte == '\n' || byte == '\r') {
            if (pos > 0) {
                buf[pos] = '\0';
                parse_command(buf);
                pos = 0;
            }
        } else if (pos < CTRL_BUF_SIZE - 1) {
            buf[pos++] = (char)byte;
        }
    }
}
