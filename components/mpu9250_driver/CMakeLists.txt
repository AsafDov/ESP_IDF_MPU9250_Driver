# CMakeLists.txt for the mpu9250_driver component

idf_component_register(SRCS "mpu9250_driver.c"
                            "kalman.c"               # List of source files to compile
                       INCLUDE_DIRS "include"              # The directory for public header files
                       REQUIRES driver freertos esp_timer )           # List of other components this driver depends on