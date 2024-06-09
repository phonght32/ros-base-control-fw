#include "BaseControl_Private.h"
#include "BaseControl.h"
#include "BaseControl_HwIntf.h"
#include "Periph.h"

/* Time update index */
#define CONTROL_MOTOR_TIME_INDEX                0       /*!< Time index control motor */
#define CMD_VEL_PUBLISH_TIME_INDEX              1       /*!< Time index publish velocity */
#define DRIVE_INFORMATION_PUBLISH_TIME_INDEX    2       /*!< Time index publish drive information */
#define IMU_PUBLISH_TIME_INDEX                  3       /*!< Time index publish IMU information */
#define IMU_UPDATE_TIME_INDEX 					4		/*!< Time index update IMU */
#define LOG_PUBLISH_TIME_INDEX                  5       /*!< Time index publish log information */

/* Frequency of publish/subscribe */
#define CONTROL_MOTOR_SPEED_FREQUENCY          	10      /*!< Frequency in Hz to control motor */
#define CONTROL_MOTOR_TIMEOUT                  	500     /*!< Period in ms to check control motor timeout */
#define IMU_PUBLISH_FREQUENCY                  	15      /*!< Frequency in Hz to publish IMU information */
#define IMU_UPDATE_FREQQUENCY 					1000	/*!< Frequency in Hz to update IMU information */
#define CMD_VEL_PUBLISH_FREQUENCY              	5       /*!< Frequency in Hz to publish robot velocity */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    	5       /*!< Frequency in Hz to publish drive information */
#define DEBUG_LOG_FREQUENCY                    	10      /*!< Frequency in Hz to send log debug messages */

uint32_t base_control_time_update[10];

err_code_t BaseControl_Init(void)
{
	/* Initialize peripherals */
	periph_imu_init();
	periph_imu_filter_init();
	periph_motor_init();
	periph_encoder_init();

	/* Initialize ROS*/
	base_control_ros_setup();

	/* Initialize base control */
	base_control_setup();

	return ERR_CODE_SUCCESS;
}

err_code_t BaseControl_Main(void)
{
	/* Update time counter */
	uint32_t t = hw_intf_get_time_ms();

	/* Update ROS time */
	base_control_update_time();

	/* Update variable */
	base_control_update_variable(base_control_connect_status());

	/* Update TF */
	base_control_update_tf_prefix(base_control_connect_status());

	/* Control motor*/
	if ((t - base_control_time_update[CONTROL_MOTOR_TIME_INDEX] >= 1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
	{
		base_control_update_goal_vel();
		if ((t - base_control_get_time_callback_cmdvel()) > CONTROL_MOTOR_TIMEOUT)
		{
			base_control_set_zero_vel();
		}
		else
		{
			base_control_set_goal_vel();
		}
		base_control_time_update[CONTROL_MOTOR_TIME_INDEX] = t;
	}

	/* Publish motor speed to "cmd_vel_motor" topic */
	if ((t - base_control_time_update[CMD_VEL_PUBLISH_TIME_INDEX]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
	{
		base_control_publish_cmdvel_from_motor_msg();
		base_control_time_update[CMD_VEL_PUBLISH_TIME_INDEX] = t;
	}

	/* Publish driver information */
	if ((t - base_control_time_update[DRIVE_INFORMATION_PUBLISH_TIME_INDEX]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
	{
		/* Publish Odom, TF and JointState, */
		base_control_publish_drive_info();
		base_control_time_update[DRIVE_INFORMATION_PUBLISH_TIME_INDEX] = t;
	}

	/* Update IMU */
	if ((t - base_control_time_update[IMU_UPDATE_TIME_INDEX]) >= (1000 / IMU_UPDATE_FREQQUENCY))
	{
		/* Publish Odom, TF and JointState, */
		base_control_update_imu();
		base_control_time_update[IMU_UPDATE_TIME_INDEX] = t;
	}

	/* Publish IMU to "imu" topic */
	if ((t - base_control_time_update[IMU_PUBLISH_TIME_INDEX]) >= (1000 / IMU_PUBLISH_FREQUENCY))
	{
		base_control_publish_imu_msg();
		base_control_time_update[IMU_PUBLISH_TIME_INDEX] = t;
	}

	/* Send log message */
	base_control_send_log_msg();

	/* Spin NodeHandle to keep synchorus */
	base_control_spin_once();

	/* Keep rosserial connection */
	base_control_wait_serial_link(base_control_connect_status());

	return ERR_CODE_SUCCESS;
}
