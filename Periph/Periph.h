// MIT License

// Copyright (c) 2023 phonght32

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

#ifndef __PERIPH_H__
#define __PERIPH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

err_code_t periph_imu_init(void);
err_code_t periph_imu_filter_init(void);
err_code_t periph_imu_get_accel(float *accel_x, float *accel_y, float* accel_z);
err_code_t periph_imu_get_gyro(float *gyro_x, float *gyro_y, float* gyro_z);
err_code_t periph_imu_update_quat(void);
err_code_t periph_imu_get_quat(float *q0, float *q1, float *q2, float* q3);

err_code_t periph_motor_init(void);
err_code_t periph_motor_left_start(void);
err_code_t periph_motor_left_stop(void);
err_code_t periph_motor_left_set_speed(float speed);
err_code_t periph_motor_right_start(void);
err_code_t periph_motor_right_stop(void);
err_code_t periph_motor_right_set_speed(float speed);

err_code_t periph_encoder_init(void);
err_code_t periph_encoder_left_get_tick(int32_t *tick);
err_code_t periph_encoder_right_get_tick(int32_t *tick);


#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_H__ */
