#include "base_control_hw_define.h"
#include "periph.h"

#define DEFAULT_ACCEL_BIAS_X				0
#define DEFAULT_ACCEL_BIAS_Y				0
#define DEFAULT_ACCEL_BIAS_Z				0

#define DEFAULT_GYRO_BIAS_X					0
#define DEFAULT_GYRO_BIAS_Y					0
#define DEFAULT_GYRO_BIAS_Z					0

#define DEFAULT_MAG_HARD_BIAS_X				0
#define DEFAULT_MAG_HARD_BIAS_Y				0
#define DEFAULT_MAG_HARD_BIAS_Z				0
#define DEFAULT_MAG_SOFT_BIAS_X				0
#define DEFAULT_MAG_SOFT_BIAS_Y				0
#define DEFAULT_MAG_SOFT_BIAS_Z				0

#define ENCODER_COUNTER_MODE_UP  			0
#define ENCODER_COUNTER_MODE_DOWN  			1

#define STEPMOTOR_PWM_DUTY  				50

mpu6050_handle_t mpu6050_handle = NULL;
imu_madgwick_handle_t imu_madgwick_handle = NULL;
stepmotor_handle_t motor_left_handle = NULL;
stepmotor_handle_t motor_right_handle = NULL;
encoder_handle_t encoder_left_handle = NULL;
encoder_handle_t encoder_right_handle = NULL;

err_code_t periph_imu_init(periph_imu_cfg_t cfg)
{
#ifdef USE_MPU6050
	/* Config MPU6500 */
	mpu6050_handle = mpu6050_init();
	if (mpu6050_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;

	mpu6050_cfg_t mpu6050_cfg = {
		.clksel = cfg.clksel,
		.dlpf_cfg = cfg.dlpf_cfg,
		.sleep_mode = cfg.sleep_mode,
		.gfs_sel = cfg.gfs_sel,
		.afs_sel = cfg.afs_sel,
		.accel_bias_x = 0,
		.accel_bias_y = 0,
		.accel_bias_z = 0,
		.gyro_bias_x = 0,
		.gyro_bias_y = 0,
		.gyro_bias_z = 0,
		.i2c_send = cfg.i2c_send,
		.i2c_recv = cfg.i2c_recv,
		.delay = cfg.delay
	};

	err_ret = mpu6050_set_config(mpu6050_handle, mpu6050_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = mpu6050_config(mpu6050_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	mpu6050_auto_calib(mpu6050_handle);
#endif
	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_filter_init(periph_imu_filter_cfg_t cfg)
{
	/* Config madgwick filter */
	err_code_t err_ret;

	imu_madgwick_handle = imu_madgwick_init();
	if (imu_madgwick_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	imu_madgwick_cfg_t imu_madgwick_cfg = {
		.beta = cfg.beta,
		.sample_freq = cfg.sample_freq
	};
	err_ret = imu_madgwick_set_config(imu_madgwick_handle, imu_madgwick_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = imu_madgwick_config(imu_madgwick_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_get_accel(float *accel_x, float *accel_y, float* accel_z)
{
	err_code_t err_ret;

#ifdef USE_MPU6050
	err_ret = mpu6050_get_accel_scale(mpu6050_handle, accel_x, accel_y, accel_z);
#endif
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_get_gyro(float *gyro_x, float *gyro_y, float* gyro_z)
{
	err_code_t err_ret;

#ifdef USE_MPU6050
	err_ret = mpu6050_get_gyro_scale(mpu6050_handle, gyro_x, gyro_y, gyro_z);
#endif
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}


	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_update_quat(void)
{
	if ((mpu6050_handle == NULL) || (imu_madgwick_handle == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;
	float accel_x, accel_y, accel_z;
	float gyro_x, gyro_y, gyro_z;

#ifdef USE_MPU6050
	err_ret = mpu6050_get_accel_scale(mpu6050_handle, &accel_x, &accel_y, &accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = mpu6050_get_gyro_scale(mpu6050_handle, &gyro_x, &gyro_y, &gyro_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}
#endif

	err_ret = imu_madgwick_update_6dof(imu_madgwick_handle, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_get_quat(float *q0, float *q1, float *q2, float* q3)
{
	if (imu_madgwick_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;

	err_ret = imu_madgwick_get_quaternion(imu_madgwick_handle, q0, q1, q2, q3);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_init(periph_motor_cfg_t cfg)
{
	err_code_t err_ret;

	/* Initialize left motor */
	motor_left_handle = stepmotor_init();
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	stepmotor_cfg_t leftmotor_cfg = {
		.dir = cfg.leftmotor_dir,
		.duty = cfg.leftmotor_duty,
		.freq_hz = cfg.leftmotor_freq_hz,
		.set_pwm_duty = cfg.leftmotor_set_pwm_duty,
		.set_pwm_freq = cfg.leftmotor_set_pwm_freq,
		.start_pwm = cfg.leftmotor_start_pwm,
		.stop_pwm = cfg.leftmotor_stop_pwm,
		.set_dir = cfg.leftmotor_set_dir
	};
	err_ret = stepmotor_set_config(motor_left_handle, leftmotor_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = stepmotor_config(motor_left_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	/* Initialize right motor */
	motor_right_handle = stepmotor_init();
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	stepmotor_cfg_t rightmotor_cfg = {
		.dir = cfg.rightmotor_dir,
		.duty = cfg.rightmotor_duty,
		.freq_hz = cfg.rightmotor_freq_hz,
		.set_pwm_duty = cfg.rightmotor_set_pwm_duty,
		.set_pwm_freq = cfg.rightmotor_set_pwm_freq,
		.start_pwm = cfg.rightmotor_start_pwm,
		.stop_pwm = cfg.rightmotor_stop_pwm,
		.set_dir = cfg.rightmotor_set_dir
	};
	err_ret = stepmotor_set_config(motor_right_handle, rightmotor_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = stepmotor_config(motor_right_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	stepmotor_set_pwm_freq(motor_left_handle, 0);
	stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_FORWARD);
	stepmotor_start(motor_left_handle);

	stepmotor_set_pwm_freq(motor_right_handle, 0);
	stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	stepmotor_set_dir(motor_right_handle, MOTORRIGHT_DIR_FORWARD);
	stepmotor_start(motor_right_handle);

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_left_start(void)
{
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_start(motor_left_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}
err_code_t periph_motor_left_stop(void)
{
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_stop(motor_left_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_left_set_speed(float speed)
{
	if (motor_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (speed < 0)
	{
		stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_BACKWARD);
		encoder_set_mode(encoder_left_handle, ENCODER_COUNTER_MODE_DOWN);
		stepmotor_set_pwm_freq(motor_left_handle, (uint32_t)(-speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	}
	else
	{
		stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_FORWARD);
		encoder_set_mode(encoder_left_handle, ENCODER_COUNTER_MODE_UP);
		stepmotor_set_pwm_freq(motor_left_handle, (uint32_t)(speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_right_start(void)
{
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_start(motor_right_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_right_stop(void)
{
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err;

	err = stepmotor_stop(motor_right_handle);
	if (err != ERR_CODE_SUCCESS)
	{
		return err;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_motor_right_set_speed(float speed)
{
	if (motor_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	if (speed < 0)
	{
		stepmotor_set_dir(motor_right_handle, MOTORLEFT_DIR_BACKWARD);
		encoder_set_mode(encoder_right_handle, ENCODER_COUNTER_MODE_DOWN);
		stepmotor_set_pwm_freq(motor_right_handle, (uint32_t)(-speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	}
	else
	{
		stepmotor_set_dir(motor_right_handle, MOTORLEFT_DIR_FORWARD);
		encoder_set_mode(encoder_right_handle, ENCODER_COUNTER_MODE_UP);
		stepmotor_set_pwm_freq(motor_right_handle, (uint32_t)(speed * VEL2FREQ));
		stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_encoder_init(periph_encoder_cfg_t cfg)
{
	err_code_t err_ret;

	/* Initialize left encoder */
	encoder_left_handle = encoder_init();
	if (encoder_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	encoder_cfg_t left_encoder_cfg = {
		.max_reload = cfg.left_encoder_max_reload,
		.start = cfg.left_encoder_start,
		.stop = cfg.left_encoder_stop,
		.set_counter = cfg.left_encoder_set_counter,
		.get_counter = cfg.left_encoder_get_counter,
		.set_mode = cfg.left_encoder_set_mode,
	};
	err_ret = encoder_set_config(encoder_left_handle, left_encoder_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = encoder_config(encoder_left_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	/* Initialize right encoder */
	encoder_right_handle = encoder_init();
	if (encoder_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	encoder_cfg_t right_encoder_cfg = {
		.max_reload = cfg.right_encoder_max_reload,
		.start = cfg.right_encoder_start,
		.stop = cfg.right_encoder_stop,
		.set_counter = cfg.right_encoder_set_counter,
		.get_counter = cfg.right_encoder_get_counter,
		.set_mode = cfg.right_encoder_set_mode,
	};
	err_ret = encoder_set_config(encoder_right_handle, right_encoder_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = encoder_config(encoder_right_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	encoder_set_mode(encoder_left_handle, ENCODER_COUNTER_MODE_UP);
	encoder_set_mode(encoder_right_handle, ENCODER_COUNTER_MODE_UP);

	encoder_start(encoder_left_handle);
	encoder_start(encoder_right_handle);

	return ERR_CODE_SUCCESS;
}

err_code_t periph_encoder_left_get_tick(int32_t *tick)
{
	if (encoder_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint32_t temp;

	encoder_get_value(encoder_left_handle, &temp);
	encoder_set_value(encoder_left_handle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	*tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

	return ERR_CODE_SUCCESS;
}

err_code_t periph_encoder_right_get_tick(int32_t *tick)
{
	if (encoder_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint32_t temp;

	encoder_get_value(encoder_right_handle, &temp);
	encoder_set_value(encoder_right_handle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	*tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

	return ERR_CODE_SUCCESS;
}

