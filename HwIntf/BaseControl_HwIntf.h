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

/*
 * @brief   MPU6050 read bytes function.
 *
 * @param   reg_addr Register address.
 * @param 	buf Buffer data.
 * @param 	len Length.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_mpu6050_read_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len);

/*
 * @brief   MPU6050 write bytes function.
 *
 * @param   reg_addr Register address.
 * @param 	buf Buffer data.
 * @param 	len Length.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_mpu6050_write_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len);

/*
 * @brief   Set PWM duty cycle of left motor.
 *
 * @param   duty Duty cycle.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_set_pwm_duty(float duty);

/*
 * @brief   Set PWM frequency of left motor.
 *
 * @param   freq Frequency.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_set_pwm_freq(uint32_t freq);

/*
 * @brief   Start motor left.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_start(void);

/*
 * @brief   Stop motor left.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_stop(void);

/*
 * @brief   Set motor left direction.
 *
 * @param   dir Direction.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_set_dir(uint8_t dir);

/*
 * @brief   Set PWM duty cycle of right motor.
 *
 * @param   duty Duty cycle.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_set_pwm_duty(float duty);

/*
 * @brief   Set PWM frequency of right motor.
 *
 * @param   freq Frequency.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_set_pwm_freq(uint32_t freq);

/*
 * @brief   Start motor right.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_start(void);

/*
 * @brief   Stop motor right.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_stop(void);

/*
 * @brief   Set motor right direction.
 *
 * @param   dir Direction.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_set_dir(uint8_t dir);

/*
 * @brief   Start left encoder.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_left_encoder_start(void);

/*
 * @brief   Stop left encoder.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_left_encoder_stop(void);

/*
 * @brief   Set encoder counter value.
 *
 * @param   value Counter value.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_left_encoder_set_counter(uint32_t value);

/*
 * @brief   Get encoder counter value.
 *
 * @param   value Counter value.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_left_encoder_get_counter(uint32_t *value);

/*
 * @brief   Set counter mode up.
 *
 * @param 	mode Counter mode. 0: Up, 1: Down.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_left_encoder_set_mode(uint8_t mode);


/*
 * @brief   Start right encoder.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_right_encoder_start(void);

/*
 * @brief   Stop right encoder.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_right_encoder_stop(void);

/*
 * @brief   Set encoder counter value.
 *
 * @param   value Counter value.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_right_encoder_set_counter(uint32_t value);

/*
 * @brief   Get encoder counter value.
 *
 * @param   value Counter value.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_right_encoder_get_counter(uint32_t *value);

/*
 * @brief   Set counter mode up.
 *
 * @param 	mode Counter mode. 0: Up, 1: Down.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_right_encoder_set_mode(uint8_t mode);

/*
 * @brief   Serial log function interface.
 *
 * @param   data Pointer data.
 * @param   len Length.
 * @param   timeout_ms Timeout in miliseconds.
 *
 * @return 	None
 */
void hw_intf_log_func(uint8_t *data, uint16_t len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __BASECONTROL_HWINTF_H__ */
