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
#include "app_camera.h"
#include "face_detection.hpp"

#include "servo.hpp"

static QueueHandle_t xQueueAIFrame = NULL;

static const char* TAG = "main";

static Servo pan_servo, tilt_servo;

class PIDController {
private:
    float Kp, Ki, Kd;
    float integral = 0;
    float prev_error = 0;
    float output_min, output_max;
    
public:
    PIDController(float p, float i, float d, float min, float max) 
        : Kp(p), Ki(i), Kd(d), output_min(min), output_max(max) {}
    
    float calculate(float setpoint, float pv, float dt) {
        float error = setpoint - pv;
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        prev_error = error;
        
        float output = Kp * error + Ki * integral + Kd * derivative;
        return std::max(output_min, std::min(output, output_max));
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
};

static PIDController pan_pid(0.006, 0.0006, 0.002, -45, 45);
static PIDController tilt_pid(0.008, 0.0006, 0.002, -45, 45);

const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

const int TARGET_X = (FRAME_WIDTH / 2) - 10;
const int TARGET_Y = FRAME_HEIGHT / 2;

static int pan_angle = 90;
static int tilt_angle = 30; 
static int pan_angle_prev = 0;
static int tilt_angle_prev = 0; 
int nose_x_prev = 0;
int nose_y_prev = 0;

void servo_control_thread()
{
    if (pan_servo.attach(12, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A) != Pwm_Error_t::PWM_OK) {
        ESP_LOGE(TAG, "Failed to attach pan servo");
        return;
    }

    if (tilt_servo.attach(13, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A) != Pwm_Error_t::PWM_OK) {
        ESP_LOGE(TAG, "Failed to attach tilt servo");
        return;
    }

    pan_servo.setAngle(pan_angle);
    tilt_servo.setAngle(tilt_angle);

    const auto loop_delay = std::chrono::milliseconds(20);
    auto last_time = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        auto results = get_last_detection_results();
        
        if (!results.empty()) {
            auto face = results.front();
            
            if (face.keypoint.size() >= 6) {
                int nose_x = face.keypoint[4];
                int nose_y = face.keypoint[5];
                
                if ((nose_x != nose_x_prev) || (nose_y != nose_y_prev))
                {
                    ESP_LOGI(TAG, "Nose at (%d, %d)", nose_x, nose_y);
                    nose_x_prev = nose_x;
                    nose_y_prev = nose_y;
                }
                
                if (abs(nose_x - TARGET_X) > 2) {
                    float pan_output = pan_pid.calculate(TARGET_X, nose_x, dt);
                    int rounded_output = static_cast<int>(pan_output + (pan_output > 0 ? 0.5f : -0.5f));
                    pan_angle += rounded_output;
                    pan_angle = std::max(0, std::min(180, pan_angle));
                    pan_servo.setAngle(pan_angle);
                }
                else {
                    pan_pid.reset();
                }

                if (abs(nose_y - TARGET_Y) > 2) {
                    float tilt_output = tilt_pid.calculate(TARGET_Y, nose_y, dt);
                    int rounded_output = static_cast<int>(tilt_output + (tilt_output > 0 ? 0.5f : -0.5f));
                    tilt_angle -= rounded_output;
                    tilt_angle = std::max(0, std::min(170, tilt_angle));
                    tilt_servo.setAngle(tilt_angle);
                }
                else {
                    tilt_pid.reset();
                }
                
                if ((pan_angle != pan_angle_prev) || (tilt_angle != tilt_angle_prev))
                {
                    ESP_LOGI(TAG, "Pan servo angle: %d", pan_angle);
                    ESP_LOGI(TAG, "Tilt servo angle: %d", tilt_angle);
                    pan_angle_prev = pan_angle;
                    tilt_angle_prev = tilt_angle;
                }
            }
        } else {
            pan_pid.reset();
            tilt_pid.reset();
        }

        std::this_thread::sleep_for(loop_delay);
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
    cfg_servo.prio = 5;
    cfg_servo.pin_to_core = 0;
    cfg_servo.thread_name = "servo_thread";

    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg_servo));

    std::thread servo_thread(servo_control_thread);
    servo_thread.detach();
}
