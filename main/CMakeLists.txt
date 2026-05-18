idf_component_register(
    SRCS
        "main.c"
        "pca9685.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_timer esp_driver_ledc esp_driver_i2c esp_driver_uart esp_driver_gpio
)
