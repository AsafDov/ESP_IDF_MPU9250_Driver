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

// MPU9250 Config
#define MPU9250_SENSOR_ADDR 0x68

// Globals:

// Functions

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

    mpu9250_handle_t mpu9250_handle = mpu9250_install(bus_handle);
    QueueHandle_t mpu9250_data_queue = mpu9250_get_data_queue(mpu9250_handle);

    while(1){
        
        /* Orientation is defined as followed in my case:
        *   - pitch: rotation around the Z-axis
        *   - roll: rotation around the X-axis
        *   - yaw: rotation around the Y-axis
        */

        quaternion_t q;
        if (xQueueReceive(mpu9250_data_queue, &q, portMAX_DELAY) == pdTRUE) {
            // Process the received orientation data
            float pitch, roll, yaw;
            quaternion_to_euler(q, &pitch, &roll, &yaw);
            ESP_LOGI("MPU DATA", "Pitch: %.0f, Roll: %.0f, Yaw: %.0f | Quaternion - w: %.2f, x: %.2f, y: %.2f, z: %.2f", pitch, roll, yaw, q.w, q.x, q.y, q.z);
            // ESP_LOGI("MPU DATA", "Quaternion - w: %.2f, x: %.2f, y: %.2f, z: %.2f", q.w, q.x, q.y, q.z);
            vTaskDelay(pdMS_TO_TICKS(10)); // Might not need to wait cause queue task is waiting
        }
    }

}
