# ESP-IDF MPU9250 Driver

[](https://opensource.org/licenses/MIT)
[](https://github.com/espressif/esp-idf)
[](https://github.com/AsafDov/ESP_IDF_MPU9250_Driver)

A robust, thread-safe MPU9250 9-DOF sensor driver for the Espressif IoT Development Framework (ESP-IDF). This driver is designed from the ground up to leverage the power of FreeRTOS, ensuring non-blocking, efficient, and reliable sensor data acquisition.

It utilizes the MPU9250's internal FIFO buffer to capture high-frequency sensor data (accelerometer, gyroscope, and magnetometer) and processes it through a Kalman filter to provide a stable orientation output in Euler angles (pitch, roll, and yaw).

-----

## ✨ Features

  * **ESP-IDF v5.x Compatible:** Built using the latest I2C master driver APIs.
  * **9-DOF Sensor Fusion:** Reads accelerometer, gyroscope, and the onboard AK8963 magnetometer.
  * **FreeRTOS-Based:** Operates on dedicated FreeRTOS tasks, making your main application logic cleaner and non-blocking.
  * **Thread-Safe:** Uses a FreeRTOS queue to safely pass processed orientation data to any consumer task.
  * **FIFO Buffer Integration:** Efficiently reads sensor data in batches using the MPU9250's hardware FIFO, reducing I2C bus traffic.
  * **Kalman Filter:** Implements a Kalman filter to fuse sensor data, minimizing noise and drift to produce a stable orientation.
  * **Easy to Integrate:** A simple `_install` function and a clear data-retrieval API.

-----

## 🏗️ Architecture

This driver's core strength lies in its multi-task architecture, which ensures high performance and thread safety.

1.  **Polling Task (`check_buffer_task`):** A lightweight, high-priority task that continuously polls the MPU9250's FIFO count register. When a sufficient number of data frames are available, it gives a semaphore to signal the processing task.
2.  **Processing Task (`mpu9250_task`):** This task remains blocked, consuming no CPU time, until it receives the semaphore. Once signaled, it performs a single I2C burst-read to retrieve the entire FIFO buffer, processes the raw data, applies the Kalman filter, and sends the final `orientation_t` data to a public queue.
3.  **Application Task (`app_main` or other):** Your main application logic can receive the processed orientation data from the queue whenever it's needed, without ever directly interacting with the I2C bus or sensor hardware.

This decoupled design, using a semaphore for synchronization and a queue for data transfer, makes the driver highly efficient and easy to integrate into complex applications.

```
+--------------------------+
|   check_buffer_task      |----(Polls FIFO Count)----> [MPU9250]
| (High Priority, 10ms)    |
+--------------------------+
            |
            | xSemaphoreGive()
            V
+--------------------------+
|   mpu9250_task           |<---(Reads FIFO Buffer)--- [MPU9250]
| (Waits on Semaphore)     |
| - Processes Raw Data     |
| - Applies Kalman Filter  |
+--------------------------+
            |
            | xQueueSend()
            V
+--------------------------+
|      Data Queue          |
|  (orientation_t)         |
+--------------------------+
            |
            | xQueueReceive()
            V
+--------------------------+
|   Your Application Task  |
| (e.g., app_main)         |
+--------------------------+
```

-----

## 💡 Sensor Fusion: Kalman Filter

To provide a stable orientation, this driver fuses data from all three sensors.

  * **Accelerometer:** Provides a reliable sense of gravity (pitch and roll) over the long term but is susceptible to noise from linear acceleration (movement).
  * **Gyroscope:** Provides excellent short-term information about the rate of rotation but suffers from drift over time.
  * **Magnetometer:** Provides a heading reference (yaw) relative to the Earth's magnetic field, but can be disturbed by local magnetic interference.

The driver uses a **Kalman filter** for each axis (pitch, roll, yaw) to optimally combine these sources. The gyroscope data is used to predict the new orientation, and the accelerometer/magnetometer data is used to correct for any accumulated drift. The result is a smooth and responsive orientation output that is more accurate than any single sensor could provide.

-----

## 🚀 Getting Started

### Prerequisites

  * ESP-IDF v5.0 or later.
  * An ESP32/S3/C3 series development board.
  * An MPU9250 sensor module connected via I2C.

### 1\. Installation

Clone this repository into the `components` directory of your ESP-IDF project:

```bash
cd your_project_directory/components
git clone https://github.com/AsafDov/ESP_IDF_MPU9250_Driver.git
```

### 2\. Configuration & Initialization

In your `app_main.c`, configure and initialize the I2C master bus, then install the MPU9250 driver.

```c
#include "driver/i2c_master.h"
#include "mpu9250_driver.h"

// I2C Configuration
#define SCL_SPEED_HZ 100000
#define SCL_PIN      GPIO_NUM_22 // Change to your SCL pin
#define SDA_PIN      GPIO_NUM_21 // Change to your SDA pin
#define I2C_PORT     0

// MPU9250 Configuration
#define MPU9250_SENSOR_ADDR 0x68

void app_main(void)
{
    // 1. Initialize I2C Master Bus
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // 2. Install the MPU9250 Driver
    mpu9250_config_t mpu9250_cfg = {
        .device_address = MPU9250_SENSOR_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = SCL_SPEED_HZ,
    };
    mpu9250_handle_t mpu9250_handle = mpu9250_init(bus_handle, &mpu9250_cfg);

    // ... rest of your application
}
```

### 3\. Reading Orientation Data

Once initialized, the driver runs in the background. To get the latest orientation data, simply retrieve the driver's data queue handle and read from it in your application's main loop.

```c
#include "mpu9250_driver.h" // Also include headers from above

void app_main(void)
{
    // ... I2C and MPU9250 initialization from step 2 ...

    // 3. Get the handle to the data queue
    QueueHandle_t mpu9250_data_queue = mpu9250_get_data_queue(mpu9250_handle);

    // 4. Create a loop to receive and process data
    while(1) {
        orientation_t orientation;

        // Wait indefinitely for new data to arrive in the queue
        if (xQueueReceive(mpu9250_data_queue, &orientation, portMAX_DELAY) == pdTRUE) {
            // New data received!
            ESP_LOGI("MPU_DATA", "Pitch: %.2f, Roll: %.2f, Yaw: %.2f",
                     orientation.pitch, orientation.roll, orientation.yaw);
        }
    }
}

```

-----

## 🔧 Status & Roadmap

This driver is currently **in active development**. The core functionality is operational, but there is more to come.

  * [x] Basic I2C communication and device initialization.
  * [x] Reading Accelerometer, Gyroscope, and Magnetometer.
  * [x] FIFO-based data acquisition.
  * [x] FreeRTOS multi-task architecture.
  * [x] Kalman filter for sensor fusion.
  * [ ] **TODO:** Implement runtime configuration for sensitivity (g-range, dps-range).
  * [ ] **TODO:** Add comprehensive sensor calibration routines (gyro bias, accel bias, magnetometer hard/soft iron).
  * [ ] **TODO:** Implement an alternative sensor fusion algorithm (e.g., Madgwick or Mahony) using quaternions.
  * [ ] **TODO:** Add support for interrupt-driven FIFO reading.
  * [ ] **TODO:** Add power management features.

-----

## 🤝 Contributing

Contributions, issues, and feature requests are welcome\! Feel free to check the [issues page](https://www.google.com/search?q=https://github.com/AsafDov/ESP_IDF_MPU9250_Driver/issues).
