#include "hw_intf.h"
#include "mpu6050.h"
#include "imu_madgwick.h"

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

#define DEFAULT_MADGWICK_BETA  				0.1f
#define DEFAULT_MADGWICK_SAMPLE_FREQ  		10000.0f

mpu6050_handle_t mpu6050_handle = NULL;
imu_madgwick_handle_t imu_madgwick_handle = NULL;

void periph_imu_init()
{
	mpu6050_handle = mpu6050_init();
	mpu6050_cfg_t mpu6050_cfg = {
		.clksel = MPU6050_CLKSEL_X_GYRO_REF,
		.dlpf_cfg = MPU6050_44ACCEL_42GYRO_BW_HZ,
		.sleep_mode = MPU6050_DISABLE_SLEEP_MODE,
		.gfs_sel = MPU6050_GFS_SEL_2000,
		.afs_sel = MPU6050_AFS_SEL_8G,
		.accel_bias_x = 0,
		.accel_bias_y = 0,
		.accel_bias_z = 0,
		.gyro_bias_x = 0,
		.gyro_bias_y = 0,
		.gyro_bias_z = 0,
		.i2c_send = hw_intf_mpu6050_write_bytes,
		.i2c_recv = hw_intf_mpu6050_read_bytes,
		.delay = hw_intf_delay_ms
	};

	mpu6050_set_config(mpu6050_handle, mpu6050_cfg);
	mpu6050_config(mpu6050_handle);
	mpu6050_auto_calib(mpu6050_handle);
}

void periph_imu_filter_init(void)
{
	/* Config madgwick filter */
	imu_madgwick_handle = imu_madgwick_init();
	imu_madgwick_cfg_t imu_madgwick_cfg = {
		.beta = DEFAULT_MADGWICK_BETA,
		.sample_freq = DEFAULT_MADGWICK_SAMPLE_FREQ
	};
	imu_madgwick_set_config(imu_madgwick_handle, imu_madgwick_cfg);
	imu_madgwick_config(imu_madgwick_handle);
}

void periph_imu_get_accel(float *accel_x, float *accel_y, float* accel_z)
{
	float temp_accel_x, temp_accel_y, temp_accel_z;

	if (MPU6050_STATUS_SUCCESS != mpu6050_get_accel_scale(mpu6050_handle, &temp_accel_x, &temp_accel_y, &temp_accel_z))
	{
		return;
	}

	*accel_x = temp_accel_x;
	*accel_y = temp_accel_y;
	*accel_z = temp_accel_z;

	return;
}

void periph_imu_get_gyro(float *gyro_x, float *gyro_y, float* gyro_z)
{
	float temp_gyro_x, temp_gyro_y, temp_gyro_z;

	if (MPU6050_STATUS_SUCCESS != mpu6050_get_gyro_scale(mpu6050_handle, &temp_gyro_x, &temp_gyro_y, &temp_gyro_z))
	{
		return;
	}

	*gyro_x = temp_gyro_x;
	*gyro_y = temp_gyro_y;
	*gyro_z = temp_gyro_z;

	return;
}

void periph_imu_update_quat(void)
{
	if ((mpu6050_handle == NULL) || (imu_madgwick_handle == NULL))
	{
		return;
	}

	float accel_x, accel_y, accel_z;
	float gyro_x, gyro_y, gyro_z;

	if (MPU6050_STATUS_SUCCESS != mpu6050_get_accel_scale(mpu6050_handle, &accel_x, &accel_y, &accel_z))
	{
		return;
	}

	if (MPU6050_STATUS_SUCCESS != mpu6050_get_gyro_scale(mpu6050_handle, &gyro_x, &gyro_y, &gyro_z))
	{
		return;
	}

	if (IMU_MADGWICK_STATUS_SUCCESS != imu_madgwick_update_6dof(imu_madgwick_handle, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z))
	{
		return;
	}

	return;
}

void periph_imu_get_quat(float *q0, float *q1, float *q2, float* q3)
{
	if (imu_madgwick_handle == NULL)
	{
		return;
	}

	float temp_q0, temp_q1, temp_q2, temp_q3;

	if (IMU_MADGWICK_STATUS_SUCCESS != imu_madgwick_get_quaternion(imu_madgwick_handle, &temp_q0, &temp_q1, &temp_q2, &temp_q3))
	{
		return;
	}

	*q0 = temp_q0;
	*q1 = temp_q1;
	*q2 = temp_q2;
	*q3 = temp_q3;

	return;
}
