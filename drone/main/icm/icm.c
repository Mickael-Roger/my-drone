#include "driver/i2c.h"
#include "icm20948.h"
#include "esp_log.h"
#include <math.h>

#define I2C_PORT        I2C_NUM_0
#define SDA_GPIO        21
#define SCL_GPIO        22
#define I2C_FREQ_HZ     400000

static const char *TAG = "ICM";
static icm20948_handle_t imu;

static esp_err_t icm_i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

esp_err_t icm_init() {
    ESP_ERROR_CHECK(icm_i2c_init());
    ESP_ERROR_CHECK(ICM20948_create(&imu, I2C_PORT, ICM20948_ADDR_SLV));
    ESP_ERROR_CHECK(ICM20948_init(&imu));
    ESP_ERROR_CHECK(ICM20948_setGyroFSR(&imu, ICM20948_GYRO_250DPS));
    ESP_ERROR_CHECK(ICM20948_setAccelFSR(&imu, ICM20948_ACCEL_2G));
    ESP_ERROR_CHECK(ICM20948_enableAKC(&imu));
    return ESP_OK;
}

esp_err_t icm_get_orientation(float *heading, float *roll, float *pitch) {
    icm20948_data_t data;
    if (ICM20948_read_all(&imu, &data) != ESP_OK) {
        return ESP_FAIL;
    }

    float ax = data.ax_g;
    float ay = data.ay_g;
    float az = data.az_g;
    float mx = data.mx_ut;
    float my = data.my_ut;
    float mz = data.mz_ut;

    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    *roll  = atan2f( ay, az ) * 180.0f / M_PI;

    float mag_x = mx * cosf(*pitch * M_PI / 180) + mz * sinf(*pitch * M_PI / 180);
    float mag_y = mx * sinf(*roll * M_PI / 180) * sinf(*pitch * M_PI / 180)
                + my * cosf(*roll * M_PI / 180)
                - mz * sinf(*roll * M_PI / 180) * cosf(*pitch * M_PI / 180);

    *heading = atan2f(-mag_y, mag_x) * 180.0f / M_PI;
    if (*heading < 0) *heading += 360.0f;

    return ESP_OK;
}

void icm_task(void *pvParameters) {

    icm_task_args_t *args = (icm_task_args_t *)pvParameters;

    while (1) {
        if (icm_get_orientation(args->heading, args->roll, args->pitch) != ESP_OK) {
            ESP_ERROR(TAG, "Could not read ICM");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelete(NULL);

}


void get_icm(float *heading, float *roll, float *pitch) {

    static icm_task_args_t icm_args = {
        .heading = heading,
        .roll = roll,
        .pitch = pitch
    };

    ESP_LOGI(TAG, "Init ICM");
    icm_init();

    ESP_LOGI(TAG, "Read ICM");

    xTaskCreate(icm_task, "icm_task", 4096, &icm_args, 5, NULL);

}





