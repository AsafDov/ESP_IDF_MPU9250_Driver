/*
    OLED LED DRIVER - Project by Asaf "Arliden The Bard" Dov
    Running random walk example. 
    adapt to your liking.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "oled_driver.h"
#include "mpu9250_driver.h"
#include "sdkconfig.h"
#include <stdlib.h> // Required for rand() and srand()
#include <time.h>   // Required for time()

#define SCL_SPEED_HZ 100000
#define SCL_PIN GPIO_NUM_22
#define SDA_PIN GPIO_NUM_21

// OLED CONFIG
#define OLED_I2C_ADDRESS 0x3c
#define OLED_SCREEN_WIDTH 128
#define OLED_SCREEN_HEIGHT 64
#define OLED_STATUS_BAR_HEIGHT 16
#define OLED_CONTENT_HEIGHT (OLED_SCREEN_HEIGHT - OLED_STATUS_BAR_HEIGHT)
#define OLED_REFRESH_RATE_MS 100

// MPU9250 Config
#define MPU9250_SENSOR_ADDR 0x68

// Globals:

// Functions
// Random walk function task
void random_walk(void* params){
    oled_handle_t oled_handle = (oled_handle_t)params;
    srand(time(NULL));
    int x = rand() % 128;
    int y = rand() % 48 + 16;
    int dir = 0;
    while (1) {
        oled_square_filled(oled_handle, x, y, x + 1, y + 1);
        vTaskDelay(oled_get_refresh_rate(oled_handle) / portTICK_PERIOD_MS);
        dir = rand() % 4; // Random direction: 0=up, 1=down, 2=left, 3=right
        switch (dir) {
            case 0: // Up
                y = ((y-16) - 1 + 48 ) % 48 + 16;
                break;
            case 1: // Down
                y = ((y-16) + 1 ) % 48 + 16;
                break;
            case 2: // Left
                x = (x - 1 + 128) % 128;
                break;
            case 3: // Right
                x = (x + 1) % 128;
                break;
        }
    }
};

oled_handle_t oled_install(i2c_master_bus_handle_t bus_handle){
    static const char *TAG = "OLED";
    /* Configure and Instantiate OLED device */
    ESP_LOGI(TAG, "Configuring OLED");
    oled_config_t oled_cfg = {
        .device_address = OLED_I2C_ADDRESS,
        .address_bit_length = I2C_ADDR_BIT_LEN_7,
        .width = OLED_SCREEN_WIDTH,
        .height = OLED_SCREEN_HEIGHT,
        .refresh_rate_ms = OLED_REFRESH_RATE_MS,
        .scl_speed_hz = SCL_SPEED_HZ,
    };
    ESP_LOGI(TAG, "Initializing OLED");
    oled_handle_t oled_handle = oled_init(bus_handle, &oled_cfg);
    ESP_LOGI(TAG, "OLED Initialized");

    return oled_handle;
};

mpu9250_handle_t mpu9250_install(i2c_master_bus_handle_t bus_handle){
    static const char *TAG = "MPU 9250";
    /* Configure and Instantiate OLED device */
    ESP_LOGI(TAG, "Configuring MPU 9250");
    mpu9250_config_t mpu9250_cfg = {
        .device_address = MPU9250_SENSOR_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = SCL_SPEED_HZ,
    };
    ESP_LOGI(TAG, "Initializing MPU");
    mpu9250_handle_t mpu9250_handle = mpu9250_init(bus_handle, &mpu9250_cfg);
    ESP_LOGI(TAG, "MPU Initialized");

    return mpu9250_handle;
};

void app_main(void)
{
    ESP_LOGI("I2C", "Configuring I2C Master");
    /* Configure and Initialize I2C Master */
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = SCL_PIN, 
        .sda_io_num = SDA_PIN, 
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_LOGI("I2C", "Initializing I2C Master");
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // oled_handle_t oled_handle = oled_install(bus_handle);

    mpu9250_handle_t mpu9250_handle = mpu9250_install(bus_handle);
    QueueHandle_t mpu9250_data_queue = mpu9250_get_data_queue(mpu9250_handle);

    while(1){
        
        /* Orientation is defined as followed in my case:
        *   - pitch: rotation around the Z-axis
        *   - roll: rotation around the X-axis
        *   - yaw: rotation around the Y-axis
        */

        orientation_t orientation;
        if (xQueueReceive(mpu9250_data_queue, &orientation, portMAX_DELAY) == pdTRUE) {
            // Process the received orientation data
            // ESP_LOGI("MPU DATA", "Pitch: %.2f, Roll: %.2f, Yaw: %.2f", orientation.pitch, orientation.roll, orientation.yaw);
            vTaskDelay(pdMS_TO_TICKS(10)); // Might not need to wait cause queue task is waiting
        }
    }


    /* Start random walk task example */
    // xTaskCreatePinnedToCore(random_walk, "random_walk", 2048, oled_handle, 5, NULL, 1);

}
