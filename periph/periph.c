#include "hw_define.h"
#include "periph.h"
#include "imu.h"
#include "madgwick/imu_madgwick.h"
#include "stepmotor.h"

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

#define RESOLVER_COUNTER_MODE_UP  			0
#define RESOLVER_COUNTER_MODE_DOWN  		1

#define STEPMOTOR_PWM_DUTY  				50

imu_handle_t imu_handle = NULL;
imu_madgwick_handle_t imu_madgwick_handle = NULL;
stepmotor_handle_t motor_left_handle = NULL;
stepmotor_handle_t motor_right_handle = NULL;
resolver_handle_t resolver_left_handle = NULL;
resolver_handle_t resolver_right_handle = NULL;

err_code_t periph_imu_init(periph_imu_cfg_t cfg)
{
	/* Config MPU6500 and AK8963 */
	imu_handle = imu_init();
	if (imu_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;

	imu_cfg_t imu_cfg = {
		.accel_bias_x = DEFAULT_ACCEL_BIAS_X,
		.accel_bias_y = DEFAULT_ACCEL_BIAS_Y,
		.accel_bias_z = DEFAULT_ACCEL_BIAS_Z,
		.gyro_bias_x = DEFAULT_GYRO_BIAS_X,
		.gyro_bias_y = DEFAULT_GYRO_BIAS_Y,
		.gyro_bias_z = DEFAULT_GYRO_BIAS_Z,
		.mag_hard_iron_bias_x = DEFAULT_MAG_HARD_BIAS_X,
		.mag_hard_iron_bias_y = DEFAULT_MAG_HARD_BIAS_Y,
		.mag_hard_iron_bias_z = DEFAULT_MAG_HARD_BIAS_Z,
		.mag_soft_iron_bias_x = DEFAULT_MAG_SOFT_BIAS_X,
		.mag_soft_iron_bias_y = DEFAULT_MAG_SOFT_BIAS_Y,
		.mag_soft_iron_bias_z = DEFAULT_MAG_SOFT_BIAS_Z,
		.func_delay = cfg.func_delay,
		.mpu6050_read_bytes = cfg.mpu6050_read_bytes,
		.mpu6050_write_bytes = cfg.mpu6050_write_bytes,
		.ak8963_read_bytes = cfg.ak8963_read_bytes,
		.ak8963_write_bytes = cfg.ak8963_write_bytes,
		.mpu6500_read_bytes = cfg.mpu6500_read_bytes,
		.mpu6500_write_bytes = cfg.mpu6500_write_bytes,
	};
	err_ret = imu_set_config(imu_handle, imu_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = imu_config(imu_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	imu_auto_calib(imu_handle);

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
	if (imu_madgwick_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;

	err_ret = imu_get_accel_scale(imu_handle, accel_x, accel_y, accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_get_gyro(float *gyro_x, float *gyro_y, float* gyro_z)
{
	if (imu_madgwick_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;

	err_ret = imu_get_gyro_scale(imu_handle, gyro_x, gyro_y, gyro_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}


	return ERR_CODE_SUCCESS;
}

err_code_t periph_imu_update_quat(void)
{
	if ((imu_handle == NULL) || (imu_madgwick_handle == NULL))
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;
	float accel_x, accel_y, accel_z;
	float gyro_x, gyro_y, gyro_z;

	err_ret = imu_get_accel_scale(imu_handle, &accel_x, &accel_y, &accel_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = imu_get_gyro_scale(imu_handle, &gyro_x, &gyro_y, &gyro_z);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

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

	stepmotor_set_pwm_duty(motor_left_handle, STEPMOTOR_PWM_DUTY);
	stepmotor_set_pwm_freq(motor_left_handle, 0);
	stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_FORWARD);
	stepmotor_start(motor_left_handle);

	stepmotor_set_pwm_duty(motor_right_handle, STEPMOTOR_PWM_DUTY);
	stepmotor_set_pwm_freq(motor_right_handle, 0);
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
		resolver_set_mode(resolver_left_handle, RESOLVER_COUNTER_MODE_DOWN);
		stepmotor_set_pwm_freq(motor_left_handle, (uint32_t)(-speed * VEL2FREQ));
	}
	else
	{
		stepmotor_set_dir(motor_left_handle, MOTORLEFT_DIR_FORWARD);
		resolver_set_mode(resolver_left_handle, RESOLVER_COUNTER_MODE_UP);
		stepmotor_set_pwm_freq(motor_left_handle, (uint32_t)(speed * VEL2FREQ));
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
		resolver_set_mode(resolver_right_handle, RESOLVER_COUNTER_MODE_DOWN);
		stepmotor_set_pwm_freq(motor_right_handle, (uint32_t)(-speed * VEL2FREQ));
	}
	else
	{
		stepmotor_set_dir(motor_right_handle, MOTORLEFT_DIR_FORWARD);
		resolver_set_mode(resolver_right_handle, RESOLVER_COUNTER_MODE_UP);
		stepmotor_set_pwm_freq(motor_right_handle, (uint32_t)(speed * VEL2FREQ));
	}

	return ERR_CODE_SUCCESS;
}

err_code_t periph_resolver_init(periph_resolver_cfg_t cfg)
{
	err_code_t err_ret;

	/* Initialize left resolver */
	resolver_left_handle = resolver_init();
	if (resolver_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	resolver_cfg_t left_resolver_cfg = {
		.max_reload = cfg.left_resolver_max_reload,
		.start = cfg.left_resolver_start,
		.stop = cfg.left_resolver_stop,
		.set_counter = cfg.left_resolver_set_counter,
		.get_counter = cfg.left_resolver_get_counter,
		.set_mode = cfg.left_resolver_set_mode,
	};
	err_ret = resolver_set_config(resolver_left_handle, left_resolver_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = resolver_config(resolver_left_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	/* Initialize right resolver */
	resolver_right_handle = resolver_init();
	if (resolver_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	resolver_cfg_t right_resolver_cfg = {
		.max_reload = cfg.right_resolver_max_reload,
		.start = cfg.right_resolver_start,
		.stop = cfg.right_resolver_stop,
		.set_counter = cfg.right_resolver_set_counter,
		.get_counter = cfg.right_resolver_get_counter,
		.set_mode = cfg.right_resolver_set_mode,
	};
	err_ret = resolver_set_config(resolver_right_handle, right_resolver_cfg);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	err_ret = resolver_config(resolver_right_handle);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	resolver_set_mode(resolver_left_handle, RESOLVER_COUNTER_MODE_UP);
	resolver_set_mode(resolver_right_handle, RESOLVER_COUNTER_MODE_UP);

	resolver_start(resolver_left_handle);
	resolver_start(resolver_right_handle);

	return ERR_CODE_SUCCESS;
}

err_code_t periph_resolver_left_get_tick(int32_t *tick)
{
	if (resolver_left_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint32_t temp;

	resolver_get_value(resolver_left_handle, &temp);
	resolver_set_value(resolver_left_handle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	*tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

	return ERR_CODE_SUCCESS;
}

err_code_t periph_resolver_right_get_tick(int32_t *tick)
{
	if (resolver_right_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	uint32_t temp;

	resolver_get_value(resolver_right_handle, &temp);
	resolver_set_value(resolver_right_handle, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);

	*tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

	return ERR_CODE_SUCCESS;
}

