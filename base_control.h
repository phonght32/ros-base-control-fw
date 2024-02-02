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

typedef uint32_t (*base_control_get_time_milisec)(void);
typedef void (*base_control_delay)(uint32_t time_ms);

/*
 * @brief   Set functions for ROS.
 *
 * @note 	This function must be called first to assign hardware interface.
 *
 * @param   get_time Function get time.
 * @param   delay Function delay.
 *
 * @return  None.
 */
void base_control_set_ros_func(base_control_get_time_milisec get_time,
                               base_control_delay delay);

/*
 * @brief   This function do the following jobs as below:
 *				- Initialize ROS node handle.
 *				- Initialize transform broadcaster.
 *				- Initialize odometry.
 *				- Initialize joint state.
 *				- Subscribe topic "cmd_vel" and "reset".
 * 				- Advertise data to topic "imu", "cmd_vel_motor", "odom" and "joint_states".
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_ros_setup(void);

/*
 * @brief   Setup robot kinematic model.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_setup(void);

/*
 * @brief  	Get connect status between base control and ROS node.
 *
 * @param   None.
 *
 * @return
 *      - TRUE:		Connected.
 *      - FALSE: 	Not connected.
 */
bool base_control_connect_status(void);

/*
 * @brief  	Spin once to keep connnection.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_spin_once(void);

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
 * @brief   Update goal velocity from topic control velocity ("cmd_vel" by default).
 *
 * @note 	Goal velocity include linear and angular velocity.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_goal_vel(void);

/*
 * @brief   Set zero velocity to motor.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_set_zero_vel(void);

/*
 * @brief   Set goal velocity to motor.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_set_goal_vel(void);

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
 * @brief   Update IMU.
 *
 * @param   None.
 *
 * @return  None.
 */
void base_control_update_imu(void);

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

/*
 * @brief   Get timestamp when callback of topic "cmd_vel" is triggered.
 *
 * @param   None.
 *
 * @return  Timestamp.
 */
uint32_t base_control_get_time_callback_cmdvel(void);

#endif /* _ROBOT_ROS_CONFIG_H_ */
