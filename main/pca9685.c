#include "pca9685.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pca9685";

#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRESCALE    0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_ALL_LED_ON  0xFA

static i2c_master_dev_handle_t dev_handle;

static esp_err_t pca_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev_handle, buf, 2, 100);
}

static esp_err_t pca_read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev_handle, &reg, 1, val, 1, 100);
}

void pca9685_init(void)
{
    // shared i2c bus
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(PCA9685_I2C_PORT, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9685_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev_handle));

    esp_err_t ret = i2c_master_probe(bus, PCA9685_I2C_ADDR, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pca9685 not found at 0x%02x: %s", PCA9685_I2C_ADDR, esp_err_to_name(ret));
        return;
    }

    // sw reset
    pca_write_reg(PCA9685_MODE1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    // sleep for prescale
    pca_write_reg(PCA9685_MODE1, 0x10);
    vTaskDelay(pdMS_TO_TICKS(5));

    // prescale = round(25MHz / (4096 * 50Hz)) - 1 = 121
    pca_write_reg(PCA9685_PRESCALE, 121);

    // wake + auto-increment
    pca_write_reg(PCA9685_MODE1, 0x20);
    vTaskDelay(pdMS_TO_TICKS(5));

    // totem-pole outputs
    pca_write_reg(PCA9685_MODE2, 0x04);

    uint8_t mode;
    pca_read_reg(PCA9685_MODE1, &mode);
    ESP_LOGI(TAG, "pca9685 ready, mode1=0x%02x, 50Hz", mode);
}

void pca9685_set_pulse(uint8_t ch, uint16_t pulse_us)
{
    // 50Hz = 20ms period, 4096 ticks/period
    // off_count = pulse_us * 4096 / 20000
    uint16_t off = (uint16_t)((uint32_t)pulse_us * 4096 / 20000);

    uint8_t reg = PCA9685_LED0_ON_L + 4 * ch;
    // on=0, off=pulse end
    uint8_t buf[5] = { reg, 0x00, 0x00, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8) };
    i2c_master_transmit(dev_handle, buf, 5, 100);
}

void pca9685_set_all_pulse(uint16_t pulse_us)
{
    uint16_t off = (uint16_t)((uint32_t)pulse_us * 4096 / 20000);

    uint8_t buf[5] = { PCA9685_ALL_LED_ON, 0x00, 0x00, (uint8_t)(off & 0xFF), (uint8_t)(off >> 8) };
    i2c_master_transmit(dev_handle, buf, 5, 100);
}
