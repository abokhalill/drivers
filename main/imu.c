#include "imu.h"
#include "config.h"
#include "shared_state.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "imu";

// mpu6050 registers
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I     0x75

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static esp_err_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev_handle, buf, 2, 100);
}

static esp_err_t imu_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 100);
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "scanning i2c bus 0x03..0x77...");
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, 50);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "  *** device at 0x%02x ***", addr);
        }
    }
    ESP_LOGI(TAG, "scan done");
}

void imu_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_SDA_PIN,
        .scl_io_num = IMU_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    ESP_LOGI(TAG, "i2c bus created on sda=GPIO%d scl=GPIO%d", IMU_SDA_PIN, IMU_SCL_PIN);

    // bus settle + power-on
    vTaskDelay(pdMS_TO_TICKS(500));

    // probe common addresses
    esp_err_t p68 = i2c_master_probe(bus_handle, 0x68, 100);
    esp_err_t p69 = i2c_master_probe(bus_handle, 0x69, 100);
    ESP_LOGI(TAG, "probe 0x68: %s (0x%x)", esp_err_to_name(p68), p68);
    ESP_LOGI(TAG, "probe 0x69: %s (0x%x)", esp_err_to_name(p69), p69);

    i2c_scan();

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = 100000,  // internal pullups too weak for 400khz
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // verify, 5 retries
    uint8_t who = 0;
    bool found = false;
    for (int i = 0; i < 5; i++) {
        if (imu_read_regs(MPU6050_WHO_AM_I, &who, 1) == ESP_OK) {
            found = true;
            break;
        }
        ESP_LOGW(TAG, "who_am_i retry %d", i + 1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (!found) {
        ESP_LOGE(TAG, "mpu6050 not responding at 0x%02x, check wiring", IMU_I2C_ADDR);
        return;
    }
    ESP_LOGI(TAG, "mpu6050 who_am_i: 0x%02x (expect 0x68)", who);

    // clear sleep bit
    if (imu_write_reg(MPU6050_PWR_MGMT_1, 0x00) != ESP_OK) {
        ESP_LOGE(TAG, "pwr_mgmt_1 write failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "mpu6050 initialized");
}

bool imu_read_raw(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t buf[14];
    if (imu_read_regs(MPU6050_ACCEL_XOUT_H, buf, 14) != ESP_OK) {
        return false;
    }

    // accel: ±2g default, 16384 LSB/g
    int16_t raw_ax = (buf[0] << 8) | buf[1];
    int16_t raw_ay = (buf[2] << 8) | buf[3];
    int16_t raw_az = (buf[4] << 8) | buf[5];
    // skip temp (buf[6], buf[7])
    int16_t raw_gx = (buf[8] << 8) | buf[9];
    int16_t raw_gy = (buf[10] << 8) | buf[11];
    int16_t raw_gz = (buf[12] << 8) | buf[13];

    *ax = raw_ax / 16384.0f;
    *ay = raw_ay / 16384.0f;
    *az = raw_az / 16384.0f;
    // gyro: ±250°/s default, 131 LSB/°/s
    *gx = raw_gx / 131.0f;
    *gy = raw_gy / 131.0f;
    *gz = raw_gz / 131.0f;

    return true;
}

void imu_compute_angles(float ax, float ay, float az, float *roll, float *pitch)
{
    // accel-only tilt; dmp later
    *roll  = atan2f(ay, az) * (180.0f / M_PI);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / M_PI);
}

void task_imu(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(10);
    TickType_t last_wake = xTaskGetTickCount();

    float ax, ay, az, gx, gy, gz;
    float roll, pitch;

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        if (!imu_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            static int fail_count = 0;
            if (++fail_count % 100 == 0)
                ESP_LOGE(TAG, "imu read failed %d times", fail_count);
            continue;
        }

        imu_compute_angles(ax, ay, az, &roll, &pitch);

        // atomic; no mutex
        atomic_store(&g_car.roll_deg, roll);
        atomic_store(&g_car.pitch_deg, pitch);
        atomic_store(&g_car.accel_z_g, az);

        // mutex-protected log
        xSemaphoreTake(g_car.mutex, portMAX_DELAY);
        uint32_t idx = g_car.log_index % 512;
        g_car.log_roll[idx]  = roll;
        g_car.log_pitch[idx] = pitch;
        g_car.log_index++;
        xSemaphoreGive(g_car.mutex);
    }
}
