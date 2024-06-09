// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __BASECONTROL_HWINTF_H__
#define __BASECONTROL_HWINTF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

uint32_t hw_intf_get_time_ms(void);
void hw_intf_delay_ms(uint32_t time_ms);
void hw_intf_log_func(uint8_t *data, uint16_t len, uint32_t timeout_ms);

err_code_t hw_intf_mpu6050_read_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len);
err_code_t hw_intf_mpu6050_write_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len);

err_code_t hw_intf_leftmotor_set_pwm_duty(float duty);
err_code_t hw_intf_leftmotor_set_pwm_freq(uint32_t freq);
err_code_t hw_intf_leftmotor_start(void);
err_code_t hw_intf_leftmotor_stop(void);
err_code_t hw_intf_leftmotor_set_dir(uint8_t dir);
err_code_t hw_intf_rightmotor_set_pwm_duty(float duty);
err_code_t hw_intf_rightmotor_set_pwm_freq(uint32_t freq);
err_code_t hw_intf_rightmotor_start(void);
err_code_t hw_intf_rightmotor_stop(void);
err_code_t hw_intf_rightmotor_set_dir(uint8_t dir);

err_code_t hw_intf_left_encoder_start(void);
err_code_t hw_intf_left_encoder_stop(void);
err_code_t hw_intf_left_encoder_set_counter(uint32_t value);
err_code_t hw_intf_left_encoder_get_counter(uint32_t *value);
err_code_t hw_intf_left_encoder_set_mode(uint8_t mode);
err_code_t hw_intf_right_encoder_start(void);
err_code_t hw_intf_right_encoder_stop(void);
err_code_t hw_intf_right_encoder_set_counter(uint32_t value);
err_code_t hw_intf_right_encoder_get_counter(uint32_t *value);
err_code_t hw_intf_right_encoder_set_mode(uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __BASECONTROL_HWINTF_H__ */
