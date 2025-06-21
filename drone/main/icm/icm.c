#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "icm20948.h"
#include "icm20948_i2c.h"


#define CONFIG_I2C_MASTER_SDA 5
#define CONFIG_I2C_MASTER_SCL 6


#define TAG "ICM"

static icm20948_device_t icm;

typedef struct {
    double *heading;
    double *roll;
    double *pitch;
} icm_task_args_t;

/* i2c bus configuration */
static const i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SDA,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = (gpio_num_t) CONFIG_I2C_MASTER_SCL,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 400000,
    .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

/* ICM 20948 configuration */
static const icm0948_config_i2c_t icm_config = {
    .i2c_port = I2C_NUM_1,
    .i2c_addr = ICM_20948_I2C_ADDR_AD0
};

static void init_dmp(icm20948_device_t *icm)
{
    bool success = true;

    success &= (icm20948_init_dmp_sensor_with_defaults(icm) == ICM_20948_STAT_OK);
    success &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_STAT_OK);
    success &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat9, 0) == ICM_20948_STAT_OK);
    success &= (icm20948_enable_fifo(icm, true) == ICM_20948_STAT_OK);
    success &= (icm20948_enable_dmp(icm, 1) == ICM_20948_STAT_OK);
    success &= (icm20948_reset_dmp(icm) == ICM_20948_STAT_OK);
    success &= (icm20948_reset_fifo(icm) == ICM_20948_STAT_OK);

    if (!success) {
        ESP_LOGE(TAG, "Enable DMP failed!");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
        ESP_LOGI(TAG, "DMP enabled!");
    }
}


void icm_init() {
    ESP_ERROR_CHECK(i2c_param_config(icm_config.i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(icm_config.i2c_port, conf.mode, 0, 0, 0));

    icm20948_init_i2c(&icm, &icm_config);

    
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK) {
        ESP_LOGE(TAG, "check id failed");
        ESP_LOGE(TAG, "%d", icm20948_check_id(&icm));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uint8_t whoami = 0x00;
    icm20948_status_e stat;
    stat = icm20948_get_who_am_i(&icm, &whoami);


    if (stat != ICM_20948_STAT_OK && whoami != ICM_20948_WHOAMI) {

        ESP_LOGE(TAG, "stat (0x%x) does not match (0x%x)", stat, ICM_20948_STAT_OK);
        ESP_LOGE(TAG, "whoami (0x%x) does not match (0x%x)", whoami, ICM_20948_WHOAMI);

    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    icm20948_sw_reset(&icm);
    vTaskDelay(pdMS_TO_TICKS(250));

    icm20948_internal_sensor_id_bm sensors = ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR;
    icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS);

    icm20948_fss_t myfss = {.a = GPM_2, .g = DPS_250};
    icm20948_set_full_scale(&icm, sensors, myfss);

    icm20948_dlpcfg_t myDLPcfg = {.a = ACC_D473BW_N499BW, .g = GYR_D361BW4_N376BW5};
    icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

    icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
    icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, false);

    icm20948_sleep(&icm, false);
    icm20948_low_power(&icm, false);

    init_dmp(&icm);
}

void icm_task(void *pvParameters) {
    icm_task_args_t *args = (icm_task_args_t *)pvParameters;

    while (1) {
        icm_20948_DMP_data_t data;
        icm20948_status_e status = inv_icm20948_read_dmp_data(&icm, &data);

        if ((status == ICM_20948_STAT_OK || status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) &&
            (data.header & DMP_header_bitmap_Quat9)) {

            //double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
            //double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
            //double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
            //double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3));
            //ESP_LOGI(TAG, "Mesure: %f %f %f %f %f", q1, q2, q3, q0, M_PI);
           
            ESP_LOGI(TAG, "Mesure: %lf %lf %lf", (double)data.Quat9.Data.Q1, (double)data.Quat9.Data.Q2, (double)data.Quat9.Data.Q3);
    
            ESP_LOGI(TAG, "get icm memory addres: %f %f %f", *(args->heading), *(args->roll), *(args->pitch));

            //*(args->roll) = atan2(2.0 * (q0*q1 + q2*q3), 1.0 - 2.0 * (q1*q1 + q2*q2)) * 180.0 / M_PI;
            //args->pitch = asin(2.0 * (q0*q2 - q3*q1)) * 180.0 / M_PI;
            //args->heading = atan2(2.0 * (q0*q3 + q1*q2), 1.0 - 2.0 * (q2*q2 + q3*q3)) * 180.0 / M_PI;
        }

        if (status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void get_icm(float *heading, float *roll, float *pitch) {


    icm_task_args_t *icm_args = malloc(sizeof(icm_task_args_t));
    if (!icm_args) {
        ESP_LOGE(TAG, "Failed to allocate memory for icm_args");
        return;
    }

    //ESP_LOGE(TAG, "get icm memory addres: 0x%08X 0x%08X 0x%08X", heading, roll, pitch);
    

    //icm_args->heading = heading;
    //icm_args->roll = roll;
    //icm_args->pitch = pitch;

    *(icm_args->heading) = 0.0;
    *(icm_args->roll) = 1.0;
    *(icm_args->pitch) = 2.0;

    ESP_LOGI(TAG, "Init ICM");
    icm_init();
    ESP_LOGI(TAG, "Read ICM");
    xTaskCreate(icm_task, "icm_task", 4096, &icm_args, 5, NULL);
}


//#include "driver/i2c.h"
//#include "icm20948.h"
//#include "icm20948_i2c.h"
//#include "esp_log.h"
//#include <math.h>
//
//#define I2C_PORT        I2C_NUM_0
//#define SDA_GPIO        21
//#define SCL_GPIO        22
//#define I2C_FREQ_HZ     400000
//
//static const char *TAG = "ICM";
//static icm20948_handle_t imu;
//
//static esp_err_t icm_i2c_init() {
//    i2c_config_t conf = {
//        .mode = I2C_MODE_MASTER,
//        .sda_io_num = SDA_GPIO,
//        .scl_io_num = SCL_GPIO,
//        .sda_pullup_en = GPIO_PULLUP_ENABLE,
//        .scl_pullup_en = GPIO_PULLUP_ENABLE,
//        .master.clk_speed = I2C_FREQ_HZ,
//    };
//    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
//    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
//}
//
//esp_err_t icm_init() {
//    ESP_ERROR_CHECK(icm_i2c_init());
//    ESP_ERROR_CHECK(ICM20948_create(&imu, I2C_PORT, ICM20948_ADDR_SLV));
//    ESP_ERROR_CHECK(ICM20948_init(&imu));
//    ESP_ERROR_CHECK(ICM20948_setGyroFSR(&imu, ICM20948_GYRO_250DPS));
//    ESP_ERROR_CHECK(ICM20948_setAccelFSR(&imu, ICM20948_ACCEL_2G));
//    ESP_ERROR_CHECK(ICM20948_enableAKC(&imu));
//    return ESP_OK;
//}
//
//esp_err_t icm_get_orientation(float *heading, float *roll, float *pitch) {
//    icm20948_data_t data;
//    if (ICM20948_read_all(&imu, &data) != ESP_OK) {
//        return ESP_FAIL;
//    }
//
//    float ax = data.ax_g;
//    float ay = data.ay_g;
//    float az = data.az_g;
//    float mx = data.mx_ut;
//    float my = data.my_ut;
//    float mz = data.mz_ut;
//
//    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
//    *roll  = atan2f( ay, az ) * 180.0f / M_PI;
//
//    float mag_x = mx * cosf(*pitch * M_PI / 180) + mz * sinf(*pitch * M_PI / 180);
//    float mag_y = mx * sinf(*roll * M_PI / 180) * sinf(*pitch * M_PI / 180)
//                + my * cosf(*roll * M_PI / 180)
//                - mz * sinf(*roll * M_PI / 180) * cosf(*pitch * M_PI / 180);
//
//    *heading = atan2f(-mag_y, mag_x) * 180.0f / M_PI;
//    if (*heading < 0) *heading += 360.0f;
//
//    return ESP_OK;
//}
//
//void icm_task(void *pvParameters) {
//
//    icm_task_args_t *args = (icm_task_args_t *)pvParameters;
//
//    while (1) {
//        if (icm_get_orientation(args->heading, args->roll, args->pitch) != ESP_OK) {
//            ESP_ERROR(TAG, "Could not read ICM");
//        }
//        vTaskDelay(pdMS_TO_TICKS(200));
//    }
//
//    vTaskDelete(NULL);
//
//}
//
//
//void get_icm(float *heading, float *roll, float *pitch) {
//
//    static icm_task_args_t icm_args = {
//        .heading = heading,
//        .roll = roll,
//        .pitch = pitch
//    };
//
//    ESP_LOGI(TAG, "Init ICM");
//    icm_init();
//
//    ESP_LOGI(TAG, "Read ICM");
//
//    xTaskCreate(icm_task, "icm_task", 4096, &icm_args, 5, NULL);
//
//}





