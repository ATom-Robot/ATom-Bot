#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "app_lcd.h"
#include "app_led.h"
#include "app_shell.h"
#include "freertos/FreeRTOS.h"

#include "mpu6050.h"

static const char *TAG = "main";

#define PIN_I2C_SDA GPIO_NUM_4
#define PIN_I2C_SCL GPIO_NUM_5

_xyz_f_st Gyro_deg = {0.0}, Acc_mmss = {0.0}, Gyro = {0.0}, Acc = {0.0};

static void mpu6050_task(void *pvParameters)
{
    mpu6050_init();
    ESP_LOGI(TAG, "connection:%d", mpu6050_test_connection());
    ESP_LOGI(TAG, "tag:%c", *mpu6050_get_tag());
    ESP_LOGI(TAG, "aux_vddio_level:%d", mpu6050_get_aux_vddio_level());
    ESP_LOGI(TAG, "rate divider:%d", mpu6050_get_rate());
    ESP_LOGI(TAG, "external_frame_sync:%d", mpu6050_get_external_frame_sync());

    ESP_LOGI(TAG, "dlpf_mode:%d", mpu6050_get_dlpf_mode());
    ESP_LOGI(TAG, "full_scale_gyro_range:%d", mpu6050_get_full_scale_gyro_range());
    printf("\n mpu6050_test_connection:%d\n", mpu6050_test_connection());

    mpu6050_set_dmp_enabled(true);

    mpu6050_acceleration_t accel =
    {
        .accel_x = 0,
        .accel_y = 0,
        .accel_z = 0,
    };
    mpu6050_rotation_t gyro =
    {
        .gyro_x = 0,
        .gyro_y = 0,
        .gyro_z = 0,
    };

    while (1)
    {

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void bsp_i2c_master_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)PIN_I2C_SDA;
    conf.scl_io_num = (gpio_num_t)PIN_I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

extern "C" void app_main()
{
    Led_Init();
    setBrightness(0.25);
    // Led_firt_light();

    AppLCD_Init();
    AppLVGL_run();

    bsp_i2c_master_init();
    xTaskCreatePinnedToCore(&mpu6050_task, "mpu6050_task", 2 * 2048, NULL, 5, NULL, 0);

    // while (1)
    // {
    //     led_rainbow();
    //     vTaskDelay(30);
    // }

    APP_Shell_loop();
}
