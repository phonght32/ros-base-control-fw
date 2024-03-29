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

#include "mpu6050/mpu6050.h"
#include "madgwick/imu_madgwick.h"
#include "stepmotor.h"
#include "encoder.h"

#include "err_code.h"

#define USE_MPU6050

#define DEFAULT_MADGWICK_BETA  				0.1f
#define DEFAULT_MADGWICK_SAMPLE_FREQ  		10000.0f

typedef struct {
#ifdef USE_MPU6050
	mpu6050_clksel_t        	clksel;         			/*!< MPU6050 clock source */
	mpu6050_dlpf_cfg_t      	dlpf_cfg;       			/*!< MPU6050 digital low pass filter (DLPF) */
	mpu6050_sleep_mode_t    	sleep_mode;     			/*!< MPU6050 sleep mode */
	mpu6050_gfs_sel_t        	gfs_sel;         			/*!< MPU6050 gyroscope full scale range */
	mpu6050_afs_sel_t       	afs_sel;        			/*!< MPU6050 accelerometer full scale range */
	mpu6050_func_i2c_send       i2c_send;        			/*!< MPU6050 send bytes */
	mpu6050_func_i2c_recv       i2c_recv;         			/*!< MPU6050 receive bytes */
	mpu6050_func_delay          delay;                 		/*!< MPU6050 delay function */
#endif
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
	uint32_t 					left_encoder_max_reload;           	/*!< Max reload value */
	encoder_func_start 			left_encoder_start;					/*!< Function start encoder */
	encoder_func_stop 			left_encoder_stop;					/*!< Function stop encoder */
	encoder_func_set_counter 	left_encoder_set_counter;			/*!< Function set counter */
	encoder_func_get_counter 	left_encoder_get_counter;			/*!< Function get counter */
	encoder_func_set_mode		left_encoder_set_mode;				/*!< Function set mode counter up */
	uint32_t 					right_encoder_max_reload;          	/*!< Max reload value */
	encoder_func_start 			right_encoder_start;				/*!< Function start encoder */
	encoder_func_stop 			right_encoder_stop;					/*!< Function stop encoder */
	encoder_func_set_counter 	right_encoder_set_counter;			/*!< Function set counter */
	encoder_func_get_counter 	right_encoder_get_counter;			/*!< Function get counter */
	encoder_func_set_mode		right_encoder_set_mode;				/*!< Function set mode counter up */
} periph_encoder_cfg_t;

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
 * @brief  	Initialize encoder.
 *
 * @param   cfg Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_encoder_init(periph_encoder_cfg_t cfg);

/*
 * @brief  	Get left encoder tick.
 *
 * @param   left_tick Left tick value.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_encoder_left_get_tick(int32_t *tick);

/*
 * @brief  	Get right encoder tick.
 *
 * @param   tick Right tick value.
 *
 * @return
 *      - ERR_CODE_SUCCESS:	Success.
 *      - Others:   		Fail.
 */
err_code_t periph_encoder_right_get_tick(int32_t *tick);


#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_H__ */
