#pragma once

// shared i2c
#define I2C_BUS_PORT        I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

// mpu6050
#define IMU_I2C_PORT        I2C_BUS_PORT
#define IMU_SDA_PIN         I2C_SDA_PIN
#define IMU_SCL_PIN         I2C_SCL_PIN
#define IMU_INT_PIN         4
#define IMU_I2C_FREQ_HZ     400000
#define IMU_I2C_ADDR        0x68

// pca9685
#define PCA9685_I2C_PORT    I2C_BUS_PORT
#define PCA9685_I2C_ADDR    0x40
#define PCA9685_CH_FL       0
#define PCA9685_CH_FR       1
#define PCA9685_CH_RL       2
#define PCA9685_CH_RR       3
#define PCA9685_CH_STEER    4

// bts7960
#define BTS_RPWM_GPIO       25      // fwd
#define BTS_LPWM_GPIO       26      // rev
#define BTS_R_EN_GPIO       18      // r_en
#define BTS_L_EN_GPIO       5       // l_en
#define BTS_PWM_FREQ_HZ     20000   // 20kHz, above audible
#define BTS_PWM_RES         LEDC_TIMER_10_BIT   // 0-1023
#define BTS_DUTY_MAX        1023

// servo us
#define SERVO_US_MIN        1000
#define SERVO_US_NEUTRAL    1500
#define SERVO_US_MAX        2000
#define SERVO_ACTIVE_RANGE  150     // ±us from neutral in active mode

// tune on hw
#define PID_KP_ROLL         18.0f
#define PID_KI_ROLL         0.5f
#define PID_KD_ROLL         4.0f
#define PID_KP_PITCH        18.0f
#define PID_KI_PITCH        0.5f
#define PID_KD_PITCH        4.0f
#define SERVO_DEADBAND_DEG  3.0f

// wifi
#define WIFI_AP_SSID        "SuspensionCar"
#define WIFI_AP_PASS        "esp32demo"
#define TELEMETRY_PORT      80
#define TELEMETRY_HZ        20

// rtos priorities
#define PRIORITY_IMU        (configMAX_PRIORITIES - 1)
#define PRIORITY_PID        (configMAX_PRIORITIES - 2)
#define PRIORITY_BT         (configMAX_PRIORITIES - 3)
#define PRIORITY_TELEM      (configMAX_PRIORITIES - 5)
