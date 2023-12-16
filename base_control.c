#include "hw_define.h"
#include "base_control.h"
#include "imu.h"
#include "madgwick/imu_madgwick.h"
#include "stepmotor.h"

imu_handle_t imu_handle = NULL;
imu_madgwick_handle_t imu_madgwick_handle = NULL;
stepmotor_handle_t leftmotor_handle = NULL;
stepmotor_handle_t rightmotor_handle = NULL;
resolver_handle_t resolver_left = NULL;
resolver_handle_t resolver_right = NULL;

err_code_t base_control_imu_init(base_control_imu_cfg_t cfg)
{
	/* Config MPU6500 and AK8963 */
	imu_handle = imu_init();
	if (imu_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;

	imu_cfg_t imu_cfg = {
		.accel_bias_x = 0,
		.accel_bias_y = 0,
		.accel_bias_z = 0,
		.gyro_bias_x = 0,
		.gyro_bias_y = 0,
		.gyro_bias_z = 0,
		.mag_hard_iron_bias_x = 0,
		.mag_hard_iron_bias_y = 0,
		.mag_hard_iron_bias_z = 0,
		.mag_soft_iron_bias_x = 0,
		.mag_soft_iron_bias_y = 0,
		.mag_soft_iron_bias_z = 0,
		.func_delay = cfg.delay,
		.mpu6050_read_bytes = cfg.mpu6050_read_bytes,
		.mpu6050_write_bytes = cfg.mpu6050_write_bytes
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

	/* Config madgwick filter */
	imu_madgwick_handle = imu_madgwick_init();
	if (imu_madgwick_handle == NULL)
	{
		return err_ret;
	}

	imu_madgwick_cfg_t imu_madgwick_cfg = {
		.beta = 0.1f,
		.sample_freq = 7000.0f
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


err_code_t base_control_imu_update_quat(void)
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

err_code_t base_control_imu_get_quat(float *q0, float *q1, float *q2, float* q3)
{
	if (imu_madgwick_handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	err_code_t err_ret;
	imu_madgwick_quat_t quat_data;

	err_ret = imu_madgwick_get_quaternion(imu_madgwick_handle, &quat_data);
	if (err_ret != ERR_CODE_SUCCESS)
	{
		return err_ret;
	}

	*q0 = quat_data.q0;
	*q1 = quat_data.q1;
	*q2 = quat_data.q2;
	*q3 = quat_data.q3;

	return ERR_CODE_SUCCESS;
}

err_code_t base_control_motor_init(base_control_motor_cfg_t cfg)
{
	err_code_t err_ret;

	/* Initialize left motor */
	leftmotor_handle = stepmotor_init();
	if (leftmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    stepmotor_cfg_t leftmotor_cfg = {
        .dir = 0,
        .duty = 0,
        .freq_hz = 0,
        .set_pwm_duty = cfg.leftmotor_set_pwm_duty,
        .set_pwm_freq = cfg.leftmotor_set_pwm_freq,
        .start_pwm = cfg.leftmotor_start_pwm,
        .stop_pwm = cfg.leftmotor_stop_pwm
    };
    err_ret = stepmotor_set_config(leftmotor_handle, leftmotor_cfg);
    if (err_ret != ERR_CODE_SUCCESS)
    {
        return err_ret;
    }

    err_ret = stepmotor_config(leftmotor_handle);
    if (err_ret != ERR_CODE_SUCCESS)
    {
        return err_ret;
    }

    /* Initialize right motor */
    rightmotor_handle = stepmotor_init();
	if (rightmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    stepmotor_cfg_t rightmotor_cfg = {
        .dir = 0,
        .duty = 0,
        .freq_hz = 0,
        .set_pwm_duty = cfg.rightmotor_set_pwm_duty,
        .set_pwm_freq = cfg.rightmotor_set_pwm_freq,
        .start_pwm = cfg.rightmotor_start_pwm,
        .stop_pwm = cfg.rightmotor_stop_pwm
    };
    err_ret = stepmotor_set_config(rightmotor_handle, rightmotor_cfg);
    if (err_ret != ERR_CODE_SUCCESS)
    {
        return err_ret;
    }

    err_ret = stepmotor_config(rightmotor_handle);
    if (err_ret != ERR_CODE_SUCCESS)
    {
        return err_ret;
    }

	return ERR_CODE_SUCCESS;
}

err_code_t base_control_motor_left_start(void)
{
	if (leftmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    err_code_t err;

    err = stepmotor_start(leftmotor_handle);
    if (err != ERR_CODE_SUCCESS)
    {
        return err;
    }

    return ERR_CODE_SUCCESS;
}
err_code_t base_control_motor_left_stop(void)
{
	if (leftmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    err_code_t err;

    err = stepmotor_stop(leftmotor_handle);
    if (err != ERR_CODE_SUCCESS)
    {
        return err;
    }

    return ERR_CODE_SUCCESS;
}

err_code_t base_control_motor_left_set_speed(float speed)
{
	if (leftmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    if (speed < 0)
    {
        stepmotor_set_dir(leftmotor_handle, MOTORLEFT_DIR_BACKWARD);
        resolver_set_mode_down(resolver_left);
        stepmotor_set_pwm_freq(leftmotor_handle, (uint32_t)(-speed * VEL2FREQ));
    }
    else
    {
        stepmotor_set_dir(leftmotor_handle, MOTORLEFT_DIR_FORWARD);
        resolver_set_mode_up(resolver_left);
        stepmotor_set_pwm_freq(leftmotor_handle, (uint32_t)(speed * VEL2FREQ));
    }

    return ERR_CODE_SUCCESS;
}

err_code_t base_control_motor_right_start(void)
{
	if (rightmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    err_code_t err;

    err = stepmotor_start(rightmotor_handle);
    if (err != ERR_CODE_SUCCESS)
    {
        return err;
    }

    return ERR_CODE_SUCCESS;
}

err_code_t base_control_motor_right_stop(void)
{
	if (rightmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    err_code_t err;

    err = stepmotor_stop(rightmotor_handle);
    if (err != ERR_CODE_SUCCESS)
    {
        return err;
    }

    return ERR_CODE_SUCCESS;
}

err_code_t base_control_motor_right_set_speed(float speed)
{
	if (rightmotor_handle == NULL)
    {
        return ERR_CODE_NULL_PTR;
    }

    if (speed < 0)
    {
        stepmotor_set_dir(rightmotor_handle, MOTORLEFT_DIR_BACKWARD);
        resolver_set_mode_down(resolver_right);
        stepmotor_set_pwm_freq(rightmotor_handle, (uint32_t)(-speed * VEL2FREQ));
    }
    else
    {
        stepmotor_set_dir(rightmotor_handle, MOTORLEFT_DIR_FORWARD);
        resolver_set_mode_up(resolver_right);
        stepmotor_set_pwm_freq(rightmotor_handle, (uint32_t)(speed * VEL2FREQ));
    }

    return ERR_CODE_SUCCESS;
}

