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

#ifndef __BASE_CONTROL_H__
#define __BASE_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "imu_intf.h"
#include "stepmotor.h"

#include "err_code.h"

typedef struct {
	imu_func_read_bytes 		mpu6050_read_bytes;			/*!< MPU6050 read bytes */
	imu_func_write_bytes 		mpu6050_write_bytes;		/*!< MPU6050 write bytes */
	imu_func_delay 				delay;						/*!< IMU delay function */
} base_control_cfg_t;

typedef struct {
	stepmotor_func_set_pwm_duty leftmotor_set_pwm_duty;		/*!< Function set PWM duty */
	stepmotor_func_set_pwm_freq leftmotor_set_pwm_freq;		/*!< Function set PWM frequency */
	stepmotor_func_start_pwm 	leftmotor_start_pwm;		/*!< Function start PWM */
	stepmotor_func_stop_pwm 	leftmotor_stop_pwm;			/*!< Function stop PWM */
	stepmotor_func_set_pwm_duty rightmotor_set_pwm_duty;	/*!< Function set PWM duty */
	stepmotor_func_set_pwm_freq rightmotor_set_pwm_freq;	/*!< Function set PWM frequency */
	stepmotor_func_start_pwm 	rightmotor_start_pwm;		/*!< Function start PWM */
	stepmotor_func_stop_pwm 	rightmotor_stop_pwm;		/*!< Function stop PWM */
} base_control_motor_cfg_t;

/*
 * @brief  	Set hardware interface functions to initialize peripherals.
 *
 * @note 	Hardware interface functions need to be assigned before any activity.
 *
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t base_control_set_hw_intf(base_control_cfg_t cfg);

/*
 * @brief  	Initialize IMU and filter.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t base_control_imu_init(void);

/*
 * @brief  	Update quaternion.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t base_control_imu_update_quat(void);

/*
 * @brief  	Get quaternion.
 *
 * @param   q0 Q0.
 * @param   q1 Q1.
 * @param   q2 Q2.
 * @param   q3 Q3.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t base_control_imu_get_quat(float *q0, float *q1, float *q2, float* q3);

/*
 * @brief  	Initialize all motor.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t base_control_motor_init(base_control_motor_cfg_t cfg);

#ifdef __cplusplus
}
#endif

#endif /* __BASE_CONTROL_H__ */
