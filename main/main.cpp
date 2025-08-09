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
#include "who_camera.h"
#include "who_human_face_detection.hpp"

#include "servo.hpp"

static QueueHandle_t xQueueAIFrame = NULL;

// using namespace std::chrono_literals;

static const char* TAG = "main";

static Servo servo1, servo2;

void servo_control_thread()
{
    if (servo1.attach(12) != Pwm_Error_t::PWM_OK) {
        std::cerr << "Failed to attach servo" << std::endl;
        return;
    }

    if (servo2.attach(13) != Pwm_Error_t::PWM_OK) {
        std::cerr << "Failed to attach servo" << std::endl;
        return;
    }

    const int angles[] = {0, 90, 180, 90};
    size_t index = 0;

    while (true) {
        int angle = angles[index];
        std::cout << "Servo angle: " << angle << " deg" << std::endl;
        servo1.setAngle(angle);
        servo2.setAngle(angle);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        index = (index + 1) % (sizeof(angles) / sizeof(angles[0]));
    }
}

extern "C" void app_main(void)
{
    xQueueAIFrame = xQueueCreate(2, sizeof(camera_fb_t *));

    register_camera(PIXFORMAT_RGB565, FRAMESIZE_QVGA, 2, xQueueAIFrame);
    register_human_face_detection(xQueueAIFrame, NULL, NULL, NULL, true);

    // Servo thread
    esp_pthread_cfg_t cfg_servo = esp_pthread_get_default_config();
    cfg_servo.stack_size = 4096;
    cfg_servo.prio = 6;
    cfg_servo.pin_to_core = 0;
    cfg_servo.thread_name = "servo_thread";

    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg_servo));

    std::thread servo_thread(servo_control_thread);
    servo_thread.detach();
}
