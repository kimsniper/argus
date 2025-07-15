/*
 * Copyright (c) 2025, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <esp_pthread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "mpu6050.hpp"
#include "servo.hpp"

using namespace std::chrono_literals;

static const char* TAG = "main";

static Servo servo;

static MPU6050::Device Dev = {
    .i2cPort = 0,
    .i2cAddress = MPU6050::I2C_ADDRESS_MPU6050_AD0_L
};

void imu_sensor_thread()
{
    // Initialize HAL layer first
    if (mpu6050_hal_init(Dev.i2cPort) == Mpu6050_Error_t::MPU6050_ERR) {
        std::cerr << "Failed to initialize I2C HAL" << std::endl;
        return;
    }

    // Create driver instance
    MPU6050::MPU6050_Driver mpu(Dev);
    
    // Initialize sensor
    auto err = mpu.Mpu6050_Init(&MPU6050::DefaultConfig);
    if (err != Mpu6050_Error_t::MPU6050_OK) {
        std::cerr << "MPU6050 initialization failed!"<< std::endl;
        return;
    }

    // Verify device ID
    uint8_t dev_id = 0;
    err = mpu.Mpu6050_GetDevideId(dev_id);
    if (err != Mpu6050_Error_t::MPU6050_OK || dev_id != MPU6050::WHO_AM_I_VAL) {
        std::cerr << "Invalid MPU6050 device ID: 0x" 
                  << std::hex << static_cast<int>(dev_id) << std::dec << std::endl;
        return;
    }

    std::cout << "MPU6050 initialized successfully. Device ID: 0x"
              << std::hex << static_cast<int>(dev_id) << std::dec << std::endl;

    // Main sensor reading loop
    while (true) {
        MPU6050::Mpu6050_AccelData_t accelData;
        MPU6050::Mpu6050_GyroData_t gyroData;

        // Read accelerometer
        err = mpu.Mpu6050_GetAccelData(accelData);
        if (err == Mpu6050_Error_t::MPU6050_OK) {
            std::cout << "Accelerometer - X: " << accelData.Accel_X << " m/s² | "
                      << "Y: " << accelData.Accel_Y << " m/s² | "
                      << "Z: " << accelData.Accel_Z << " m/s²" << std::endl;
        } else {
            std::cerr << "Accelerometer read error: " << static_cast<int>(err) << std::endl;
        }

        // Read gyroscope
        err = mpu.Mpu6050_GetGyroData(gyroData);
        if (err == Mpu6050_Error_t::MPU6050_OK) {
            std::cout << "Gyroscope - X: " << gyroData.Gyro_X << " °/s | "
                      << "Y: " << gyroData.Gyro_Y << " °/s | "
                      << "Z: " << gyroData.Gyro_Z << " °/s" << std::endl;
        } else {
            std::cerr << "Gyroscope read error: " << static_cast<int>(err) << std::endl;
        }

        std::cout << "----------------------------------------" << std::endl;
        std::this_thread::sleep_for(500ms);
    }
}

void servo_control_thread()
{
    if (servo.attach(0) != Pwm_Error_t::PWM_OK) {
        std::cerr << "Failed to attach servo" << std::endl;
        return;
    }

    const int angles[] = {0, 90, 180, 90};
    size_t index = 0;

    while (true) {
        int angle = angles[index];
        std::cout << "Servo angle: " << angle << " deg" << std::endl;
        servo.setAngle(angle);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        index = (index + 1) % (sizeof(angles) / sizeof(angles[0]));
    }
}

extern "C" void app_main(void)
{

    // IMU thread
    esp_pthread_cfg_t cfg_imu = esp_pthread_get_default_config();
    cfg_imu.stack_size = 4096;
    cfg_imu.prio = 5;
    cfg_imu.pin_to_core = 0;  // or 0 / 1 to pin to a specific core tskNO_AFFINITY -- any core
    cfg_imu.thread_name = "imu_thread";

    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg_imu));

    // Servo thread
    esp_pthread_cfg_t cfg_servo = esp_pthread_get_default_config();
    cfg_servo.stack_size = 4096;
    cfg_servo.prio = 5;
    cfg_servo.pin_to_core = 1;  // or 0 / 1 to pin to a specific core tskNO_AFFINITY -- any core
    cfg_servo.thread_name = "servo_thread";

    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg_servo));

    // Launch the C++ threads
    std::thread servo_thread(servo_control_thread);
    servo_thread.detach();  // Let it run independently

    std::thread imu_thread(imu_sensor_thread);
    imu_thread.detach();  // Let it run independently
}
