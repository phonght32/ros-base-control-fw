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

#include "imu.h"
#include "madgwick/imu_madgwick.h"
#include "stepmotor.h"
#include "resolver.h"

#include "err_code.h"

#define DEFAULT_MADGWICK_BETA  				0.1f
#define DEFAULT_MADGWICK_SAMPLE_FREQ  		6000.0f

typedef struct {
	imu_func_read_bytes 		mpu6050_read_bytes;			/*!< MPU6050 read bytes */
	imu_func_write_bytes 		mpu6050_write_bytes;		/*!< MPU6050 write bytes */
	imu_func_read_bytes     	ak8963_read_bytes;			/*!< AK8963 write bytes */
	imu_func_write_bytes    	ak8963_write_bytes;			/*!< AK8963 write bytes */
	imu_func_read_bytes     	mpu6500_read_bytes;			/*!< MPU6500 write bytes */
	imu_func_write_bytes    	mpu6500_write_bytes;		/*!< MPU6500 write bytes */
	imu_func_delay 				func_delay;					/*!< IMU delay function */
} periph_imu_cfg_t;

typedef struct {
	float beta;
	float sample_freq;
} periph_imu_filter_cfg_t;

typedef struct {
	uint8_t                		leftmotor_dir;            	/*!< Direction */
	uint32_t            		leftmotor_freq_hz;        	/*!< PWM frequency in Hz */
	float 						leftmotor_duty;				/*!< PWM duty cycle */
	stepmotor_func_set_pwm_duty leftmotor_set_pwm_duty;		/*!< Function set PWM duty */
	stepmotor_func_set_pwm_freq leftmotor_set_pwm_freq;		/*!< Function set PWM frequency */
	stepmotor_func_start_pwm 	leftmotor_start_pwm;		/*!< Function start PWM */
	stepmotor_func_stop_pwm 	leftmotor_stop_pwm;			/*!< Function stop PWM */
	stepmotor_func_set_dir 		leftmotor_set_dir;			/*!< Function set direction */
	uint8_t                		rightmotor_dir;            	/*!< Direction */
	uint32_t            		rightmotor_freq_hz;        	/*!< PWM frequency in Hz */
	float 						rightmotor_duty;			/*!< PWM duty cycle */
	stepmotor_func_set_pwm_duty rightmotor_set_pwm_duty;	/*!< Function set PWM duty */
	stepmotor_func_set_pwm_freq rightmotor_set_pwm_freq;	/*!< Function set PWM frequency */
	stepmotor_func_start_pwm 	rightmotor_start_pwm;		/*!< Function start PWM */
	stepmotor_func_stop_pwm 	rightmotor_stop_pwm;		/*!< Function stop PWM */
	stepmotor_func_set_dir 		rightmotor_set_dir;			/*!< Function set direction */
} periph_motor_cfg_t;

typedef struct {
	uint32_t 					left_resolver_max_reload;           /*!< Max reload value */
	resolver_func_start 		left_resolver_start;				/*!< Function start resolver */
	resolver_func_stop 			left_resolver_stop;					/*!< Function stop resolver */
	resolver_func_set_counter 	left_resolver_set_counter;			/*!< Function set counter */
	resolver_func_get_counter 	left_resolver_get_counter;			/*!< Function get counter */
	resolver_func_set_mode		left_resolver_set_mode;				/*!< Function set mode counter up */
	uint32_t 					right_resolver_max_reload;          /*!< Max reload value */
	resolver_func_start 		right_resolver_start;				/*!< Function start resolver */
	resolver_func_stop 			right_resolver_stop;				/*!< Function stop resolver */
	resolver_func_set_counter 	right_resolver_set_counter;			/*!< Function set counter */
	resolver_func_get_counter 	right_resolver_get_counter;			/*!< Function get counter */
	resolver_func_set_mode		right_resolver_set_mode;			/*!< Function set mode counter up */
} periph_resolver_cfg_t;

/*
 * @brief  	Initialize IMU.
 *
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_imu_init(periph_imu_cfg_t cfg);

/*
 * @brief  	Initialize IMU filter.
 *
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_imu_filter_init(periph_imu_filter_cfg_t cfg);

/*
 * @brief  	Get scaled accelerometer.
 *
 * @param   accel_x Accelerometer x axis.
 * @param   accel_y Accelerometer y axis.
 * @param   accel_z Accelerometer z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_imu_get_accel(float *accel_x, float *accel_y, float* accel_z);

/*
 * @brief  	Get scaled gyroscope.
 *
 * @param   gyro_x Gyroscope x axis.
 * @param   gyro_y Gyroscope y axis.
 * @param   gyro_z Gyroscope z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_imu_get_gyro(float *gyro_x, float *gyro_y, float* gyro_z);

/*
 * @brief  	Update quaternion.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_imu_update_quat(void);

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
err_code_t periph_imu_get_quat(float *q0, float *q1, float *q2, float* q3);

/*
 * @brief  	Initialize all motor.
 *
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_init(periph_motor_cfg_t cfg);

/*
 * @brief  	Start motor left.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_left_start(void);

/*
 * @brief  	Stop motor left.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_left_stop(void);

/*
 * @brief  	Set speed motor left.
 *
 * @param   speed Speed in m/s.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_left_set_speed(float speed);

/*
 * @brief  	Start motor right.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_right_start(void);

/*
 * @brief  	Stop motor right.
 *
 * @param   None.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_right_stop(void);

/*
 * @brief  	Set speed motor right.
 *
 * @param   speed Speed in m/s.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_motor_right_set_speed(float speed);

/*
 * @brief  	Initialize resolver.
 *
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_resolver_init(periph_resolver_cfg_t cfg);

/*
 * @brief  	Get left resolver tick.
 *
 * @param   left_tick Left tick value.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_resolver_left_get_tick(int32_t *tick);

/*
 * @brief  	Get right resolver tick.
 *
 * @param   tick Right tick value.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_resolver_right_get_tick(int32_t *tick);


#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_H__ */
