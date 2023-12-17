// MIT License

// Copyright (c) 2020 phonght32

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

#ifndef _ROBOT_ROS_CONFIG_H_
#define _ROBOT_ROS_CONFIG_H_

#include "ros.h"
#include "ros/time.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

/* Time update index */
#define CONTROL_MOTOR_TIME_INDEX                0       /*!< Time index control motor */
#define CMD_VEL_PUBLISH_TIME_INDEX              1       /*!< Time index publish velocity */
#define DRIVE_INFORMATION_PUBLISH_TIME_INDEX    2       /*!< Time index publish drive information */
#define IMU_PUBLISH_TIME_INDEX                  3       /*!< Time index publish IMU information */
#define LOG_PUBLISH_TIME_INDEX                  5
#define CONTROL_MOTOR_TIMEOUT_TIME_INDEX        6       /*!< Time index control motor timeout */

/* Frequency of publish/subscribe */
#define CONTROL_MOTOR_SPEED_FREQUENCY          10       /*!< Frequency in Hz to control motor */
#define CONTROL_MOTOR_TIMEOUT                  500      /*!< Period in ms to check control motor timeout */
#define IMU_PUBLISH_FREQUENCY                  15      	/*!< Frequency in Hz to publish IMU information */
#define CMD_VEL_PUBLISH_FREQUENCY              5       	/*!< Frequency in Hz to publish robot velocity */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    5       	/*!< Frequency in Hz to publish drive information */
#define DEBUG_LOG_FREQUENCY                    10       /*!< Frequency in Hz to send log debug messages */

/* Linear & Angular velocity index */
#define WHEEL_NUM       2                       /*!< Num wheel */

#define LEFT            0                       /*!< Left wheel index */
#define RIGHT           1                       /*!< Right wheel index */

#define LINEAR          0                       /*!< Linear velocity index */
#define ANGULAR         1                       /*!< Angular velocity index */

typedef uint32_t (*base_control_get_time_milisec)(void);

/*
 * @brief   Set get time function for ROS.
 *
 * @note 	Function get time need to be assigned first.
 *
 * @param   get_time Function get time.
 *
 * @return  None.
 */
void base_control_set_ros_func(base_control_get_time_milisec get_time);

/*
 * @brief   ROS setup node handle, rosserial connection, ...
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_ros_setup(void);

/*
 * @brief   Initialize Odometry.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_init_odom(void);

/*
 * @brief   Initialize JointStates
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_init_joint_state(void);

/*
 * @brief   Get ROS time current.
 *
 * @param   None.
 *
 * @return  ROS time.
 */
ros::Time base_control_get_ros_time(void);

/*
 * @brief   Add microsecond to ROS time.
 *
 * @param   t Ros time.
 * @param   _micros Time add.
 *
 * @return  ROS time calibration.
 */
ros::Time base_control_ros_time_add_microsec(ros::Time &t, uint32_t _micros);

/*
 * @brief   Update ROS time from system time.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_time(void);

/*
 * @brief   Update variable (Odometry and IMU parameters).
 *
 * @param   isConnected Check rosserial connect.
 *
 * @return  None.
 */
void base_control_update_variable(bool isConnected);

/*
 * @brief   Update TFPrefix.
 *
 * @param   isConnected Check rosserial connect.
 *
 * @return  None.
 */
void base_control_update_tf_prefix(bool isConnected);

/*
 * @brief   Update goal velocity to control motor.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_goal_vel(void);

/*
 * @brief   Update Odometry.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_odom(void);

/*
 * @brief   Update Joint.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_joint(void);

/*
 * @brief   Update JointStates.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_joint_state(void);

/*
 * @brief   Update Transform.
 *
 * @param   odom_tf Geometry messages content tf information.
 *
 * @return  None.
 */
void base_control_update_tf(geometry_msgs::TransformStamped& odom_tf);

/*
 * @brief   Update IMU bias value.
 *
 * @param   isConnected Check rosserial connect.
 *
 * @return  None.
 */
void base_control_update_gyro_cali(bool isConnected);

/*
 * @brief   Update motor information.
 *
 * @param   left_tick
 * @pram    right_tick
 *
 * @return  None.
 */
void base_control_update_motor_info(int32_t left_tick, int32_t right_tick);

/*
 * @brief   Control robot adapt to linear and angular velocity.
 *
 * @param   goal_vel Linear and angular velocity.
 *
 * @return  None.
 */
void base_control_set_vel(float *goal_vel);

/*
 * @brief   Get motor speed.
 *
 * @param   vel Pointer data.
 *
 * @return  None.
 */
void base_control_get_motor_speed(float *vel);

/*
 * @brief   Publish linear and angular velocity to "cmd_vel_motor" topic.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_publish_cmdvel_from_motor_msg(void);

/*
 * @brief   Publish drive information.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_publish_drive_info(void);

/*
 * @brief   Publish IMU information.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_publish_imu_msg(void);

/*
 * @brief   Wait rosserial connection.
 *
 * @param   isConnected Check rosserial connect.
 *
 * @return  None.
 */
void base_control_wait_serial_link(bool isConnected);

/*
 * @brief   Send Log messages.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_send_log_msg(void);

#endif /* _ROBOT_ROS_CONFIG_H_ */
