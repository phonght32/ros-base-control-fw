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

#ifndef __PERIPH_MOTOR_H__
#define __PERIPH_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

void periph_motor_init(void);
void periph_motor_left_start(void);
void periph_motor_left_stop(void);
void periph_motor_left_set_speed(float speed);
void periph_motor_right_start(void);
void periph_motor_right_stop(void);
void periph_motor_right_set_speed(float speed);

void periph_encoder_init(void);
void periph_encoder_left_get_tick(int32_t *tick);
void periph_encoder_right_get_tick(int32_t *tick);

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_MOTOR_H__ */
