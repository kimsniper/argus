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

#include "servo.hpp"

using namespace std::chrono_literals;

static const char* TAG = "main";

static Servo servo;


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
    // Optional: configure thread attributes
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 4096;
    cfg.prio = 5;
    cfg.pin_to_core = tskNO_AFFINITY;  // or 0 / 1 to pin to a specific core
    cfg.thread_name = "servo_thread";

    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));

    // Launch the C++ thread
    std::thread servo_thread(servo_control_thread);
    servo_thread.detach();  // Let it run independently
}
