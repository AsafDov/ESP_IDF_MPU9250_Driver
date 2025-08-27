#include <math.h>
#include "driver/i2c.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "mpu9250_driver.h"
#include "mpu9240_register_map.h"
#include "kalman.h"

#define TAG "MPU9250"
#define I2C_MASTER_TIMEOUT_MS       1000

typedef struct{
    float pitch_from_accel;
    float pitch_from_gyro;
    float roll_from_accel;
    float roll_from_gyro;
    float yaw_from_mag;
    float yaw_from_gyro;
    uint64_t dt;
}angles_buf_t;

struct mpu{
    i2c_master_bus_handle_t bus_handle;
    i2c_addr_bit_len_t dev_addr_length;         /*!< Select the address length of the slave device. */
    uint16_t device_address;                    /*!< I2C device raw address. (The 7/10 bit address without read/write bit). Macro I2C_DEVICE_ADDRESS_NOT_USED (0xFFFF) stands for skip the address config inside driver. */
    uint32_t scl_speed_hz;                      /*!< I2C SCL line frequency. */
    uint32_t scl_wait_us;                      /*!< Timeout value. (unit: us). Please note this value should not be so small that it can handle stretch/disturbance properly. If 0 is set, that means use the default reg value*/
    uint16_t accel_sensitivity;                 /*!< Accelerometer sensitivity setting. */
    angles_buf_t angles_buf;
    KalmanFilter roll_filter;
    KalmanFilter pitch_filter;
    KalmanFilter yaw_filter;
    struct {
        uint32_t disable_ack_check:      1;     /*!< Disable ACK check. If this is set false, that means ack check is enabled, the transaction will be stopped and API returns error when nack is detected. */
    } flags;      
    i2c_master_dev_handle_t i2c_device_mpu9250_handle; 
    QueueHandle_t data_queue;
    SemaphoreHandle_t data_semaphore;
    
};

orientation_t process_fifo_data(mpu9250_handle_t handle, uint8_t *fifo_data){
    // TODO make sensitivity more generalized
    int sensitivity = MPU9250_ACCEL_SENSITIVITY_4G;
    orientation_t orientation = {
        .pitch = 0, 
        .roll = 0,
        .yaw = 0
    };

    // Process the FIFO data to extract sensor readings
    // --- Process Accelerometer Data ---
    int16_t accel_x = (int16_t)((fifo_data[0] << 8)  | fifo_data[1]);
    int16_t accel_y = (int16_t)((fifo_data[2] << 8)  | fifo_data[3]);
    int16_t accel_z = (int16_t)((fifo_data[4] << 8)  | fifo_data[5]);
    
    // --- Process Gyroscope Data ---
    int16_t gyro_x = (int16_t)((fifo_data[6] << 8)  | fifo_data[7]);
    int16_t gyro_y = (int16_t)((fifo_data[8] << 8)  | fifo_data[9]);
    int16_t gyro_z = (int16_t)((fifo_data[10] << 8) | fifo_data[11]);
    
    // --- Process Magnetometer data 
    int16_t mag_x_raw = (int16_t)((fifo_data[13] << 8) | fifo_data[12]);
    int16_t mag_y_raw = (int16_t)((fifo_data[15] << 8) | fifo_data[14]);
    int16_t mag_z_raw = (int16_t)((fifo_data[17] << 8) | fifo_data[16]);

    int8_t mag_status = fifo_data[6];

    float mag_x_uT = (float)mag_x_raw * 0.6; // Convert to microteslas
    float mag_y_uT = (float)mag_y_raw * 0.6; // Convert to microteslas
    float mag_z_uT = (float)mag_z_raw * 0.6; // Convert to microteslas

    // Convert raw values to 'g's using floating point and the stored sensitivity
    float accel_x_g = (float)accel_x / sensitivity;
    float accel_y_g = (float)accel_y / sensitivity;
    float accel_z_g = (float)accel_z / sensitivity;
    
    float gyro_x_dps = gyro_x / MPU9250_GYRO_SENSITIVITY_500dps;
    float gyro_y_dps = gyro_y / MPU9250_GYRO_SENSITIVITY_500dps;
    float gyro_z_dps = gyro_z / MPU9250_GYRO_SENSITIVITY_500dps;

    /* Orientation is defined as followed in my case:
        *   - pitch: rotation around the Z-axis
        *   - roll: rotation around the X-axis
        *   - yaw: rotation around the Y-axis 
        * 
        * Gravity in my system is (0,-g,0) when breadboard is normally on table
    */
    // orientation.pitch = atan2f(accel_x_g, sqrtf(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180.0f / M_PI; // Around the Z axis
    
    /* Pitch from Accel*/
    float pitch_rad = atan2f(-accel_x_g, -accel_y_g);
    handle->angles_buf.pitch_from_accel = pitch_rad * 180.0f / M_PI; // Around the Z axis

    /* Roll from Accel*/
    float roll_rad = atan2f(-accel_z_g, -accel_y_g);
    handle->angles_buf.roll_from_accel  = roll_rad * 180.0f / M_PI; // Around the X axis

    
    /* Gyro integration */
    uint64_t current_time_us = esp_timer_get_time();
    // Calculate dt in seconds by finding the difference and converting from microseconds
    float dt = (float)(current_time_us - handle->angles_buf.dt) / 1000000.0f;
    // Update the last time for the next iteration
    handle->angles_buf.dt = current_time_us;
    
    /* Apply Kalman Filter */
    
    kalman_predict(&handle->pitch_filter, gyro_z_dps, dt);
    kalman_predict(&handle->roll_filter, gyro_x_dps, dt);
    kalman_predict(&handle->yaw_filter, gyro_y_dps, dt);
    
    kalman_update(&handle->pitch_filter, handle->angles_buf.pitch_from_accel);
    kalman_update(&handle->roll_filter, handle->angles_buf.roll_from_accel);
    
    /* Yaw From Magnetometer
    * For the Yaw we need to first project the magnetometer data onto a hotizontal X Z plane 
    */
    float mag_comp_y = mag_y_uT * cos(handle->roll_filter.angle * M_PI / 180.0f) - mag_z_uT * sin(handle->roll_filter.angle * M_PI / 180.0f);
    float mag_comp_z = mag_y_uT * sin(handle->roll_filter.angle * M_PI / 180.0f) * sin(handle->pitch_filter.angle * M_PI / 180.0f) + mag_x_uT * cos(handle->pitch_filter.angle * M_PI / 180.0f) + mag_z_uT * cos(handle->roll_filter.angle * M_PI / 180.0f) * sin(handle->pitch_filter.angle * M_PI / 180.0f);
    handle->angles_buf.yaw_from_mag = atan2(mag_comp_z, mag_comp_y) * 180.0f / M_PI;
    kalman_update(&handle->yaw_filter, handle->angles_buf.yaw_from_mag);

    orientation.pitch = handle->pitch_filter.angle;
    orientation.roll  = handle->roll_filter.angle;
    orientation.yaw   = handle->yaw_filter.angle;

    // ESP_LOGI("Orientation", "pitch from gyro: %.2f, roll from gyro: %.2f, yaw from gyro: %.2f", handle->angles_buf.pitch_from_gyro, handle->angles_buf.roll_from_gyro, handle->angles_buf.yaw_from_gyro);
    // ESP_LOGI("RAW MPU DATA", "Accel: [%.2f, %.2f, %.2f], Gyro: [%.2f, %.2f, %.2f], Mag: [%.2f, %.2f, %.2f]", accel_x_g, accel_y_g, accel_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps, mag_x_uT, mag_y_uT, mag_z_uT);
    ESP_LOGI("Orientation", "Pitch: %.1f, Roll: %.1f, Yaw: %.1f", orientation.pitch, orientation.roll, orientation.yaw);

    return orientation;
};
void mpu9250_task(void* mpu9250_handle){
    mpu9250_handle_t handle = (mpu9250_handle_t)mpu9250_handle;
    int fifo_size = 19; 
    // Read the FIFO buffer NO INTERRUPT version
    uint8_t fifo_data[fifo_size];
    uint8_t write_buffer = MPU9250_FIFO_R_W_REG_ADDR;
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(handle->data_semaphore, portMAX_DELAY) == pdTRUE) {
            // Read the FIFO data
            ESP_ERROR_CHECK(i2c_master_transmit_receive(handle->i2c_device_mpu9250_handle, &write_buffer, 1, fifo_data, fifo_size, -1));

            // Process the FIFO data
            orientation_t orientation = process_fifo_data(handle, fifo_data);

            // Send to queue
            xQueueSend(handle->data_queue, &orientation, 0);

        }
    }
}

void check_buffer_task(void* mpu9250_handle){
    mpu9250_handle_t handle = (mpu9250_handle_t)mpu9250_handle;
    uint8_t count_buffer[2];
    uint16_t count = 0;
    uint8_t write_buffer = MPU9250_FIFO_COUNTH_REG_ADDR;
    while (1) {
        // Read the FIFO count
        ESP_ERROR_CHECK(i2c_master_transmit_receive(handle->i2c_device_mpu9250_handle, &write_buffer, 1, count_buffer, 2, -1));
        count = (count_buffer[0] << 8) | count_buffer[1];
        
        if (count >= 19){
            xSemaphoreGive(handle->data_semaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

QueueHandle_t mpu9250_get_data_queue(mpu9250_handle_t mpu_handle){
    if (mpu_handle) {
        return mpu_handle->data_queue;
    }
    return NULL;
}

void mpu9250_send_command(mpu9250_handle_t mpu9250_handle, unsigned char* cmd, int cmd_size){
    ESP_ERROR_CHECK(i2c_master_transmit(mpu9250_handle->i2c_device_mpu9250_handle, cmd, cmd_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS));
    vTaskDelay(pdMS_TO_TICKS(20));
}

void configure_magnetometer(mpu9250_handle_t mpu_handle){
    uint8_t write_buf[2] = {0x00};
    uint8_t data = 0x00;

    /* Configure Magnetometer */
    ESP_LOGI(TAG, "Configuring Magnetometer");

    /* Enable Bypass */
    ESP_LOGI(TAG, "Enabling Bypass");
    write_buf[0] = MPU9250_INT_PIN_CFG_REG_ADDR;
    write_buf[1] = 0x02;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    vTaskDelay(pdMS_TO_TICKS(50));

    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_MAG_DEVICE_ADDR, // This is 0x0C
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t mag_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mpu_handle->bus_handle, &mag_cfg, &mag_handle));


    // --- 1. DIAGNOSTIC: Check Who Am I register ---
    uint8_t wia_reg = MPU9250_MAG_WIA_REG_ADDR; // WIA register is at 0x00
    uint8_t wia_val = 0;
    esp_err_t err = i2c_master_transmit_receive(mag_handle, &wia_reg, 1, &wia_val, 1, pdMS_TO_TICKS(100));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AK8963 WIA (Who Am I): 0x%02X (should be 0x48)", wia_val);
        if (wia_val != 0x48) {
            ESP_LOGE(TAG, "Invalid WIA value for AK8963. Check I2C connections and pull-up resistors.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read AK8963 WIA register. Magnetometer not responding.");
    }

    // --- 2. ROBUST INIT: Power Down magnetometer first ---
    ESP_LOGI(TAG, "Powering down magnetometer to ensure known state.");
    uint8_t mag_cmd[2];
    mag_cmd[0] = MPU9250_MAG_CNTL1_REG_ADDR;
    mag_cmd[1] = 0x00; // Power-down mode
    i2c_master_transmit(mag_handle, mag_cmd, 2, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Set MODE of Magnetometer */
    mag_cmd[0] = MPU9250_MAG_CNTL1_REG_ADDR;
    mag_cmd[1] = MPU9250_MAG_MODE_CONTINUOUS_100HZ;
    err = i2c_master_transmit(mag_handle, mag_cmd, 2, 1000 / portTICK_PERIOD_MS);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Successfully sent command to magnetometer!");
    } else {
        ESP_LOGE(TAG, "Failed to communicate with magnetometer. Still getting NACK?");
    }

    /* Remove I2C Device*/
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(mag_handle));

    /* Disable Bypass ????*/
    write_buf[0] = MPU9250_INT_PIN_CFG_REG_ADDR;
    write_buf[1] = 0x00;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Configure Magnetometer */
    ESP_LOGI(TAG, "Configuring I2C Master for Magnetometer read...");

    //? Maybe should | with 0x80 to read?
    /* Set Slave0 Address to be the Magnetometer */
    write_buf[0] = MPU9250_I2C_SLV0_ADDR_REG_ADDR ;
    write_buf[1] = MPU9250_MAG_DEVICE_ADDR | 0x80;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));

    /* Set Address to begin data transfer from */
    write_buf[0] = MPU9250_I2C_SLV0_REG_ADDR;
    write_buf[1] = MPU9250_MAG_HXL_REG_ADDR;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));

    //? Maybe should see if slave0 enable should be 0?
    /* Set Slave0 Enable and Number of Bytes to Read (7) */
    write_buf[0] = MPU9250_I2C_SLV0_CTRL_REG_ADDR;
    write_buf[1] = 0x87;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
}

void send_configuration_commands(mpu9250_handle_t mpu_handle){

    // --- ADD THIS DIAGNOSTIC CODE AT THE VERY BEGINNING ---
    uint8_t who_am_i_reg = MPU9250_WHO_AM_I_REG_ADDR; // This is register 0x75
    uint8_t device_id = 0;
    i2c_master_transmit_receive(mpu_handle->i2c_device_mpu9250_handle, &who_am_i_reg, 1, &device_id, 1, -1);
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, "MPU WHO_AM_I register is: 0x%02X", device_id);
    if (device_id == 0x71) {
        ESP_LOGI(TAG, "Device is a genuine MPU9250. The magnetometer might be faulty.");
    } else if (device_id == 0x70) {
        ESP_LOGE(TAG, "Device is an MPU6500. This is a 6-axis sensor with NO MAGNETOMETER.");
    } else if (device_id == 0x68) {
         ESP_LOGE(TAG, "Device is an MPU6050. This is a 6-axis sensor with NO MAGNETOMETER.");
    } else {
        ESP_LOGW(TAG, "Device ID is unrecognized. It is likely a clone without a magnetometer.");
    }
    ESP_LOGI(TAG, "======================================================");
    // --- END OF DIAGNOSTIC CODE ---

    ESP_LOGI(TAG, "Sending configuration commands to MPU9250");
    // Send the necessary configuration commands to the MPU9250
    uint8_t write_buf[2] = {0x00};
    uint8_t data = 0x00;
    
    /* Wake up the MPU9250 */
    ESP_LOGI(TAG, "Waking up MPU9250");
    data = 0x80;
    write_buf[0] = MPU9250_PWR_MGMT_1_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, 2);
    vTaskDelay(pdMS_TO_TICKS(100));

    data = 0x01;
    write_buf[0] = MPU9250_PWR_MGMT_1_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, 2);

    /* Enable Gyroscope and Accelerometer */
    ESP_LOGI(TAG, "Enabling Gyroscope and Accelerometer");
    data = 0x00;
    write_buf[0] = MPU9250_PWR_MGMT_2_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));

    
    /* Configure Gyroscope */
    ESP_LOGI(TAG, "Configuring Gyroscope");
    data = 0x08; // Set gyro config to +-500 degrees/sec
    write_buf[0] = MPU9250_GYRO_CONFIG_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    /* Configure Accelerometer */
    ESP_LOGI(TAG, "Configuring Accelerometer");
    data = 0x08; // Set accel config to +-4g
    write_buf[0] = MPU9250_ACCEL_CONFIG_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    /* Configure DLPF for Gyro */
    //! Was 0x03 for replacing buffer info with new info.
    //! Setting to 0x43 to allow buffer full to not overwrite old values
    ESP_LOGI(TAG, "Configuring DLPF for Gyro");
    data = 0x43; // Set DLPF to 41Hz meaning 5.9ms delay between gyro reads. Higher value is higher delay
    write_buf[0] = MPU9250_CONFIG_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    /* Configure DLPF for Accel */
    ESP_LOGI(TAG, "Configuring DLPF for Accel");
    data = 0x03; // Set DLPF to 44.8Hz meaning 4.88ms delay between accel reads.
    write_buf[0] = MPU9250_ACCEL_CONFIG_2_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    /* Set Sample Rate for DLPF */
    ESP_LOGI(TAG, "Setting Sample Rate for DLPF");
    data = 0x09; // Set sample rate to 200Hz
    write_buf[0] = MPU9250_SMPLRT_DIV_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    configure_magnetometer(mpu_handle);
    
    /* Enable FIFO register and Reset sensor registers and signal paths */
    ESP_LOGI(TAG, "Resetting and Enabling FIFO");
    write_buf[0] = MPU9250_USER_CTRL_REG_ADDR;
    write_buf[1] = 0x60; 
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    // /* Configure FIFO Enable Register for accelerometer*/
    //! For the purpose of debugging, this to enables only accelerometer read
    // ESP_LOGI(TAG, "Configuring FIFO Enable Register");
    // // OLD VALUE: data = 0x78;
    // data = 0x08; // FIX: Enable FIFO for Accelerometer ONLY. Disable Gyro.
    // write_buf[0] = MPU9250_FIFO_EN_REG_ADDR;
    // write_buf[1] = data;
    // mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    // /* Configure FIFO Enable Register for gyro */
    // //! For the purpose of debugging, this to enables only gyro read
    // ESP_LOGI(TAG, "Configuring FIFO Enable Register");
    // // OLD VALUE: data = 0x78;
    // data = 0x70; // FIX: Enable FIFO for Gyro ONLY. Disable Accelerometer.
    // write_buf[0] = MPU9250_FIFO_EN_REG_ADDR;
    // write_buf[1] = data;
    // mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    // //! ONLY MAGNETOMETER
    // /* Configure FIFO Enable Register ONLY FOR MAGNETOMETER*/ 
    // ESP_LOGI(TAG, "Configuring FIFO Enable Register");
    // data = 0x01; // Enable FIFO for Magnetometer ONLY. Disable Gyro and Accelerometer.
    // write_buf[0] = MPU9250_FIFO_EN_REG_ADDR;
    // write_buf[1] = data;
    // mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
    /* Configure FIFO Enable Register for accelerometer and gyro and Magnetometer*/ 
    ESP_LOGI(TAG, "Configuring FIFO Enable Register");
    data = 0x79; // Enable FIFO for Gyro and Accelerometer. Disable slv, and temp output 0x79 to enable slave0 (Magnetometer) into FIFO
    write_buf[0] = MPU9250_FIFO_EN_REG_ADDR;
    write_buf[1] = data;
    mpu9250_send_command(mpu_handle, write_buf, sizeof(write_buf));
    
}
mpu9250_handle_t mpu9250_init(i2c_master_bus_handle_t bus_handle, mpu9250_config_t* mpu9250_cfg){
    // Allocate memory for the main driver structure
    mpu9250_handle_t handle = (mpu9250_handle_t)malloc(sizeof(struct mpu));
    if (handle == NULL) {
        ESP_LOGI(TAG, "MPU9250 handle malloc failed");
        return NULL;
    }

    handle->bus_handle = bus_handle;
    handle->device_address = mpu9250_cfg->device_address;
    handle->dev_addr_length = mpu9250_cfg->dev_addr_length;
    handle->scl_speed_hz = mpu9250_cfg->scl_speed_hz;
    handle->scl_wait_us = mpu9250_cfg->scl_wait_us;
    handle->angles_buf.pitch_from_gyro = 0.0f;
    handle->angles_buf.roll_from_gyro = 0.0f;
    handle->angles_buf.yaw_from_mag = 0.0f;
    handle->angles_buf.dt = esp_timer_get_time();

    // Configure the I2C device settings
    i2c_device_config_t i2c_device_mpu9250_cfg = {
        .dev_addr_length = handle->dev_addr_length,
        .device_address = handle->device_address,
        .scl_speed_hz = handle->scl_speed_hz,
    };
    
    // Initialize the I2C device handle
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_device_mpu9250_cfg, &handle->i2c_device_mpu9250_handle));
    ESP_LOGI(TAG, "MPU9250 device added to I2C bus");
    
    // Initialize Kalman filters
    kalman_init(&handle->pitch_filter);
    kalman_init(&handle->roll_filter);
    kalman_init(&handle->yaw_filter);

    // initialize data queue
    handle->data_queue = xQueueCreate(10, sizeof(uint8_t[3])); // Pitch, Roll, Yaw
    handle->data_semaphore = xSemaphoreCreateBinary();
    
    // Configure the MPU9250
    send_configuration_commands(handle);

    // Create Sensor Reading Task
    xTaskCreatePinnedToCore(mpu9250_task, "MPU9250_task", 2600, handle, 5, NULL, 1);
    xTaskCreatePinnedToCore(check_buffer_task, "Check_Data_Buffer_Task", 2600, handle, 5, NULL, 1);


    return handle;
}

uint16_t mpu_get_raw_angle(mpu9250_handle_t mpu_handle){
    return 0;
}
uint16_t mpu_get_filtered_angle(mpu9250_handle_t mpu_handle){
    return 0;
}