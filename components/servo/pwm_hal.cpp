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

#include "pwm_hal.hpp"

Pwm_Error_t PwmHal::init(uint8_t gpio_num, uint32_t freq_hz, uint32_t resolution_hz,
                         mcpwm_unit_t mcpwm_unit, mcpwm_timer_t mcpwm_timer,
                         mcpwm_io_signals_t mcpwm_signal) {
    unit = mcpwm_unit;
    timer = mcpwm_timer;
    io_signal = mcpwm_signal;
    return setup(gpio_num, freq_hz, resolution_hz);
}

Pwm_Error_t PwmHal::setPulseWidth(uint32_t pulse_width_us) {
    return setCompareValue(pulse_width_us);
}

Pwm_Error_t PwmHal::setup(uint8_t gpio_num, uint32_t freq_hz, uint32_t resolution_hz) {
    // Configure GPIO
    mcpwm_gpio_init(unit, io_signal, gpio_num);

    // Configure PWM with all fields initialized
    mcpwm_config_t pwm_config = {
        .frequency = freq_hz,
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .cmpr_b = 0,     // duty cycle of PWMxB = 0
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    
    if (mcpwm_init(unit, timer, &pwm_config) != ESP_OK) {
        return Pwm_Error_t::PWM_ERROR;
    }
    
    return Pwm_Error_t::PWM_OK;
}

Pwm_Error_t PwmHal::setCompareValue(uint32_t pulse_width_us) {
    if (mcpwm_set_duty_in_us(unit, timer, MCPWM_OPR_A, pulse_width_us) != ESP_OK) {
        return Pwm_Error_t::PWM_ERROR;
    }
    return Pwm_Error_t::PWM_OK;
}
