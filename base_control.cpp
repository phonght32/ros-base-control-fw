#include "stdio.h"
#include "stdbool.h"

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

#include "periph/periph.h"
#include "base_control_hw_define.h"
#include "base_control.h"

#define ROS_TOPIC_IMU               "imu"
#define ROS_TOPIC_JOINT_STATES      "joint_states"
#define ROS_TOPIC_ODOM              "odom"
#define ROS_TOPIC_RESET             "reset"
#define ROS_TOPIC_CMD_VEL           "cmd_vel"
#define ROS_TOPIC_CMD_VEL_MOTOR     "cmd_vel_motor"


/* Linear & Angular velocity index */
#define WHEEL_NUM       2                       /*!< Num wheel */

#define LEFT            0                       /*!< Left wheel index */
#define RIGHT           1                       /*!< Right wheel index */

#define LINEAR          0                       /*!< Linear velocity index */
#define ANGULAR         1                       /*!< Angular velocity index */

static void base_control_init_joint_state(void);
static void base_control_init_odom(void);

static void base_control_update_gyro_cali(bool isConnected);
static void base_control_update_joint_state(void);
static void base_control_update_joint(void);
static void base_control_update_odom(void);
static void base_control_update_tf(geometry_msgs::TransformStamped& odom_tf);
static bool base_control_calc_odom(float diff_time);
static sensor_msgs::Imu base_control_get_imu(void);

static ros::Time base_control_ros_time_add_microsec(ros::Time &t, uint32_t _micros);
static ros::Time base_control_get_ros_time(void);

static void base_control_callback_cmd_vel(const geometry_msgs::Twist& cmd_vel_msg);
static void base_control_callback_reset(const std_msgs::Empty &reset_msg);

base_control_get_time_milisec get_time_milis = NULL;

uint32_t base_control_time_update[10];

ros::NodeHandle RosNodeHandle;                          /*!< ROS node handle */
ros::Time Ros_CurrentTime;                              /*!< ROS current time */
uint32_t Ros_CurrentTimeOffset;                         /*!< ROS current time offset */
unsigned long Ros_PrevUpdateTime;                       /*!< ROS previous update time */
char Ros_LogBuffer[100];                                /*!< ROS log message buffer */

bool BaseControl_InitEncoder = true;                    /*!< Base control initialize encoder flag */
float BaseControl_OdomPose[3];
float BaseControl_OdomVel[3];

char odom_header_frame_id[30];
char odom_child_frame_id[30];
char imu_frame_id[30];
char joint_state_header_frame_id[30];

sensor_msgs::Imu imu_msg;                           /*!< ROS IMU message */
geometry_msgs::Twist cmd_vel_motor_msg;             /*!< ROS command velocity message */
nav_msgs::Odometry odom;                            /*!< ROS odometry message */
sensor_msgs::JointState joint_states;               /*!< ROS joint states message */
geometry_msgs::TransformStamped odom_tf;            /*!< ROS transform stamped message */
tf::TransformBroadcaster tf_broadcaster;            /*!< ROS tf broadcaster message */

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub(ROS_TOPIC_CMD_VEL, base_control_callback_cmd_vel);
ros::Subscriber<std_msgs::Empty> reset_sub(ROS_TOPIC_RESET, base_control_callback_reset);

ros::Publisher imu_pub(ROS_TOPIC_IMU, &imu_msg);
ros::Publisher cmd_vel_motor_pub(ROS_TOPIC_CMD_VEL_MOTOR, &cmd_vel_motor_msg);
ros::Publisher odom_pub(ROS_TOPIC_ODOM, &odom);
ros::Publisher joint_states_pub(ROS_TOPIC_JOINT_STATES, &joint_states);

char get_prefix[10];
char *get_tf_prefix = get_prefix;

float goal_velocity[2] = {0.0, 0.0};                /*!< Velocity to control motor */
float goal_velocity_from_cmd[2] = {0.0, 0.0};       /*!< Velocity receive from "cmd_vel" topic */
float goal_velocity_from_motor[2] = {0.0, 0.0};     /*!< Velocity read from encoder */

int32_t BaseControl_LastDiffTick[WHEEL_NUM] = {0, 0};
float BaseControl_LastRad[WHEEL_NUM] = {0.0, 0.0};
float  BaseControl_LastVel[WHEEL_NUM]  = {0.0, 0.0};


static uint32_t millis(void)
{
    return get_time_milis();
}

static float constrain(float x, float low_val, float high_val)
{
    float value;
    if (x > high_val)
    {
        value = high_val;
    }
    else if (x < low_val)
    {
        value = low_val;
    }
    else
    {
        value = x;
    }
    return value;
}

static void base_control_callback_cmd_vel(const geometry_msgs::Twist& cmd_vel_msg)
{
    /* Get goal velocity */
    goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    /* Constrain velocity */
    goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    /* Update time */
    base_control_time_update[CONTROL_MOTOR_TIMEOUT_TIME_INDEX] = millis();
}

static void base_control_callback_reset(const std_msgs::Empty &reset_msg)
{
    char log_msg[50];

    (void)(reset_msg);

    sprintf(log_msg, "Start Calibration of Gyro");
    RosNodeHandle.loginfo(log_msg);

    base_control_init_odom();

    sprintf(log_msg, "Reset Odometry");
    RosNodeHandle.loginfo(log_msg);
}

static ros::Time base_control_get_ros_time(void)
{
    return RosNodeHandle.now();
}

static ros::Time base_control_ros_time_add_microsec(ros::Time & t, uint32_t _micros)
{
    uint32_t sec, nsec;

    sec  = _micros / 1000 + t.sec;
    nsec = _micros % 1000000000 + t.nsec;

    return ros::Time(sec, nsec);
}

static void base_control_update_odom(void)
{
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id  = odom_child_frame_id;

    odom.pose.pose.position.x = BaseControl_OdomPose[0];
    odom.pose.pose.position.y = BaseControl_OdomPose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(BaseControl_OdomPose[2]);

    odom.twist.twist.linear.x  = BaseControl_OdomVel[0];
    odom.twist.twist.angular.z = BaseControl_OdomVel[2];
}

static void base_control_update_joint_state(void)
{
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

    joint_states_pos[LEFT]  = BaseControl_LastRad[LEFT];
    joint_states_pos[RIGHT] = BaseControl_LastRad[RIGHT];

    joint_states_vel[LEFT]  = BaseControl_LastVel[LEFT];
    joint_states_vel[RIGHT] = BaseControl_LastVel[RIGHT];

    joint_states.position = (double *)joint_states_pos;
    joint_states.velocity = (double *)joint_states_vel;
}

static void base_control_update_joint(void)
{

}

static void base_control_update_tf(geometry_msgs::TransformStamped& odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

static void base_control_update_gyro_cali(bool isConnected)
{
    static bool isEnded = false;
    char log_msg[50];

    (void)(isConnected);

    if (RosNodeHandle.connected())
    {
        if (isEnded == false)
        {
            sprintf(log_msg, "Start Calibration of Gyro");
            RosNodeHandle.loginfo(log_msg);

            //calibrationGyro();

            sprintf(log_msg, "Calibration End");
            RosNodeHandle.loginfo(log_msg);

            isEnded = true;
        }
    }
    else
    {
        isEnded = false;
    }
}

static void base_control_init_odom(void)
{
    BaseControl_InitEncoder = true;

    for (int index = 0; index < 3; index++)
    {
        BaseControl_OdomPose[index] = 0.0;
        BaseControl_OdomVel[index]  = 0.0;
    }

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;

    odom.twist.twist.linear.x  = 0.0;
    odom.twist.twist.angular.z = 0.0;
}

static void base_control_init_joint_state(void)
{
    static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name            = joint_states_name;

    joint_states.name_length     = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length   = WHEEL_NUM;
}

static bool base_control_calc_odom(float diff_time)
{
    float wheel_l, wheel_r;      // rotation value of wheel [rad]
    float delta_s, theta, delta_theta;
    static float last_theta = 0.0f;
    float v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
    float step_time;

    wheel_l = wheel_r = 0.0f;
    delta_s = delta_theta = theta = 0.0f;
    v = w = 0.0f;
    step_time = 0.0f;

    step_time = diff_time;

    if (step_time == 0)
        return false;

    wheel_l = TICK2RAD * (float)BaseControl_LastDiffTick[LEFT];
    wheel_r = TICK2RAD * (float)BaseControl_LastDiffTick[RIGHT];

    if (isnan(wheel_l))
        wheel_l = 0.0f;

    if (isnan(wheel_r))
        wheel_r = 0.0f;

    delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0f;
//     theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;

    float q0, q1, q2, q3;
    periph_imu_get_quat(&q0, &q1, &q2, &q3);
    theta = atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
    delta_theta = theta - last_theta;

    // compute odometric pose
    BaseControl_OdomPose[0] += delta_s * cos(BaseControl_OdomPose[2] + (delta_theta / 2.0));
    BaseControl_OdomPose[1] += delta_s * sin(BaseControl_OdomPose[2] + (delta_theta / 2.0));
    BaseControl_OdomPose[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    BaseControl_OdomVel[0] = v;
    BaseControl_OdomVel[1] = 0.0;
    BaseControl_OdomVel[2] = w;

    BaseControl_LastVel[LEFT]  = wheel_l / step_time;
    BaseControl_LastVel[RIGHT] = wheel_r / step_time;
    last_theta = theta;

    return true;
}

static sensor_msgs::Imu base_control_get_imu(void)
{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float q0, q1, q2, q3;

    periph_imu_get_accel(&accel_x, &accel_y, &accel_z);
    periph_imu_get_gyro(&gyro_x, &gyro_y, &gyro_z);
    periph_imu_get_quat(&q0, &q1, &q2, &q3);

    sensor_msgs::Imu imu_msg_;

    imu_msg_.angular_velocity.x = gyro_x;
    imu_msg_.angular_velocity.y = gyro_y;
    imu_msg_.angular_velocity.z = gyro_z;

    imu_msg_.linear_acceleration.x = accel_x;
    imu_msg_.linear_acceleration.y = accel_y;
    imu_msg_.linear_acceleration.z = accel_z;

    imu_msg_.orientation.w = q0;
    imu_msg_.orientation.x = q1;
    imu_msg_.orientation.y = q2;
    imu_msg_.orientation.z = q3;

    imu_msg_.angular_velocity_covariance[1] = 0;
    imu_msg_.angular_velocity_covariance[2] = 0;
    imu_msg_.angular_velocity_covariance[3] = 0;
    imu_msg_.angular_velocity_covariance[4] = 0.02;
    imu_msg_.angular_velocity_covariance[5] = 0;
    imu_msg_.angular_velocity_covariance[6] = 0;
    imu_msg_.angular_velocity_covariance[7] = 0;
    imu_msg_.angular_velocity_covariance[8] = 0.02;

    imu_msg_.linear_acceleration_covariance[0] = 0.04;
    imu_msg_.linear_acceleration_covariance[1] = 0;
    imu_msg_.linear_acceleration_covariance[2] = 0;
    imu_msg_.linear_acceleration_covariance[3] = 0;
    imu_msg_.linear_acceleration_covariance[4] = 0.04;
    imu_msg_.linear_acceleration_covariance[5] = 0;
    imu_msg_.linear_acceleration_covariance[6] = 0;
    imu_msg_.linear_acceleration_covariance[7] = 0;
    imu_msg_.linear_acceleration_covariance[8] = 0.04;

    imu_msg_.orientation_covariance[0] = 0.0025;
    imu_msg_.orientation_covariance[1] = 0;
    imu_msg_.orientation_covariance[2] = 0;
    imu_msg_.orientation_covariance[3] = 0;
    imu_msg_.orientation_covariance[4] = 0.0025;
    imu_msg_.orientation_covariance[5] = 0;
    imu_msg_.orientation_covariance[6] = 0;
    imu_msg_.orientation_covariance[7] = 0;
    imu_msg_.orientation_covariance[8] = 0.0025;

    return imu_msg_;
}

void base_control_set_ros_func(base_control_get_time_milisec get_time)
{
    get_time_milis = get_time;
}

void base_control_ros_setup(void)
{
    RosNodeHandle.initNode();                       /*!< Init ROS node handle */

    RosNodeHandle.subscribe(cmd_vel_sub);           /*!< Subscribe "cmd_vel" topic to get motor cmd */
    RosNodeHandle.subscribe(reset_sub);             /*!< Subscribe "reset" topic */

    RosNodeHandle.advertise(imu_pub);               /*!< Register the publisher to "imu" topic */
    RosNodeHandle.advertise(cmd_vel_motor_pub);     /*!< Register the publisher to "cmd_vel_motor" topic */
    RosNodeHandle.advertise(odom_pub);              /*!< Register the publisher to "odom" topic */
    RosNodeHandle.advertise(joint_states_pub);      /*!< Register the publisher to "joint_states" topic */

    tf_broadcaster.init(RosNodeHandle);             /*!< Init TransformBroadcaster */
    base_control_init_odom();                       /*!< Init odometry value */
    base_control_init_joint_state();                /*!< Init joint state */

    Ros_PrevUpdateTime = millis();                    /*!< Update time */
}

bool base_control_connect_status(void)
{
    return RosNodeHandle.connected();
}

void base_control_spin_once(void)
{
    RosNodeHandle.spinOnce();
}

void base_control_update_time(void)
{
    Ros_CurrentTimeOffset = millis();
    Ros_CurrentTime = RosNodeHandle.now();
}

void base_control_update_variable(bool isConnected)
{
    static bool variable_flag = false;

    if (isConnected)
    {
        if (variable_flag == false)
        {
            /*  initIMU() */
            base_control_init_odom();

            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}

void base_control_update_tf_prefix(bool isConnected)
{
    static bool isChecked = false;
    char log_msg[60];

    if (isConnected)
    {
        if (isChecked == false)
        {
            RosNodeHandle.getParam("~tf_prefix", &get_tf_prefix);

            if (!strcmp(get_tf_prefix, ""))
            {
                sprintf(odom_header_frame_id, "odom");
                sprintf(odom_child_frame_id, "base_footprint");

                sprintf(imu_frame_id, "imu_link");
                sprintf(joint_state_header_frame_id, "base_link");
            }
            else
            {
                strcpy(odom_header_frame_id, get_tf_prefix);
                strcpy(odom_child_frame_id, get_tf_prefix);

                strcpy(imu_frame_id, get_tf_prefix);
                strcpy(joint_state_header_frame_id, get_tf_prefix);

                strcat(odom_header_frame_id, "/odom");
                strcat(odom_child_frame_id, "/base_footprint");

                strcat(imu_frame_id, "/imu_link");
                strcat(joint_state_header_frame_id, "/base_link");
            }

            sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
            RosNodeHandle.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
            RosNodeHandle.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
            RosNodeHandle.loginfo(log_msg);

            isChecked = true;
        }
    }
    else
    {
        isChecked = false;
    }
}

void base_control_update_goal_vel(void)
{
    goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
    goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

void base_control_update_motor_info(int32_t left_tick, int32_t right_tick)
{
    if (BaseControl_InitEncoder)
    {
        for (int index = 0; index < WHEEL_NUM; index++)
        {
            BaseControl_LastDiffTick[index] = 0;
            BaseControl_LastRad[index]       = 0.0f;

            BaseControl_LastVel[index]  = 0.0f;
        }

        BaseControl_InitEncoder = false;
        return;
    }

    BaseControl_LastDiffTick[LEFT] = left_tick;
    BaseControl_LastRad[LEFT] += TICK2RAD * (float)BaseControl_LastDiffTick[LEFT];

    BaseControl_LastDiffTick[RIGHT] = right_tick;
    BaseControl_LastRad[RIGHT] += TICK2RAD * (float)BaseControl_LastDiffTick[RIGHT];
}

void base_control_set_zero_vel(void)
{
    periph_motor_left_set_speed(0);
    periph_motor_right_set_speed(0);
}

void base_control_set_goal_vel(void)
{
    float wheel_velocity_cmd[2];

    float lin_vel = goal_velocity[LINEAR];
    float ang_vel = goal_velocity[ANGULAR];

    wheel_velocity_cmd[LEFT]  = lin_vel - (ang_vel * WHEEL_SEPARATION / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * WHEEL_SEPARATION / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);

    periph_motor_left_set_speed(wheel_velocity_cmd[LEFT]);
    periph_motor_right_set_speed(wheel_velocity_cmd[RIGHT]);
}

void base_control_get_motor_speed(void)
{
    goal_velocity_from_motor[LINEAR] = goal_velocity_from_cmd[LINEAR];
    goal_velocity_from_motor[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

void base_control_publish_cmdvel_from_motor_msg(void)
{
    /* Get motor velocity */
    cmd_vel_motor_msg.linear.x = goal_velocity_from_motor[LINEAR];
    cmd_vel_motor_msg.angular.z = goal_velocity_from_motor[ANGULAR];

    /* Publish veloctiy to "cmd_vel_motor" topic */
    cmd_vel_motor_pub.publish(&cmd_vel_motor_msg);
}

void base_control_publish_drive_info(void)
{
    /* Update time */
    unsigned long time_now = millis();
    unsigned long step_time = time_now - Ros_PrevUpdateTime;
    Ros_PrevUpdateTime = time_now;
    ros::Time stamp_now = base_control_get_ros_time();

    /* Calculate odometry */
    base_control_calc_odom((float)(step_time * 0.001f));

    /* Publish odometry message */
    base_control_update_odom();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    /* Publish TF message */
    base_control_update_tf(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    /* Publish jointStates message */
    base_control_update_joint_state();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
}

void base_control_publish_imu_msg(void)
{
    /* Get IMU data (accelerometer, gyroscope, quaternion and variance ) */
    imu_msg = base_control_get_imu();

    imu_msg.header.stamp = base_control_get_ros_time();
    imu_msg.header.frame_id = imu_frame_id;

    /* Publish IMU messages */
    imu_pub.publish(&imu_msg);
}

void base_control_wait_serial_link(bool isConnected)
{
    static bool wait_flag = false;

    if (isConnected)
    {
        if (wait_flag == false)
        {
            HAL_Delay(10);

            wait_flag = true;
        }
    }
    else
    {
        wait_flag = false;
    }
}

void base_control_send_log_msg(void)
{
    static bool log_flag = false;

    if (RosNodeHandle.connected())
    {
        if (log_flag == false)
        {
            sprintf(Ros_LogBuffer, "--------------------------");
            RosNodeHandle.loginfo(Ros_LogBuffer);

            sprintf(Ros_LogBuffer, "Connected to openSTM32-Board");
            RosNodeHandle.loginfo(Ros_LogBuffer);

            sprintf(Ros_LogBuffer, "--------------------------");
            RosNodeHandle.loginfo(Ros_LogBuffer);

            log_flag = true;
        }
    }
    else
    {
        log_flag = false;
    }
}
