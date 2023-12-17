#include "stdio.h"
#include "stdbool.h"

#include "periph.h"
#include "hw_define.h"
#include "base_control.h"

/*
 * Log message buffer.
 */
char log_msg[100];

/*
 * ROS NodeHandle.
 */
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*
 * ROS parameters.
 */
char get_prefix[10];
char *get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];

char joint_state_header_frame_id[30];

/*
 * Subscribers.
 */
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<std_msgs::Empty> reset_sub("reset", resetCallback);

/*
 * Publishers.
 */
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

geometry_msgs::Twist cmd_vel_motor_msg;
ros::Publisher cmd_vel_motor_pub("cmd_vel_motor", &cmd_vel_motor_msg);

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*
 * Transform Broadcaster.
 */
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*
 * Software Timer of Robot.
 */
static uint32_t tTime[10];

/*
 * Calculation for odometry.
 */
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
float last_rad[WHEEL_NUM] = {0.0, 0.0};

/*
 * Update Joint State.
 */
float  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*
 * Declaration for controllers.
 */
float zero_velocity[WHEEL_NUM] = {0.0, 0.0};            /*!< Velocity to stop motor                 */
float goal_velocity[WHEEL_NUM] = {0.0, 0.0};            /*!< Velocity to control motor              */
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};   /*!< Velocity receive from "cmd_vel" topic  */
float goal_velocity_from_motor[WHEEL_NUM] = {0.0, 0.0}; /*!< Velocity read from encoder             */

/*
 * Declaration for SLAM and Navigation.
 */
unsigned long prev_update_time;
float odom_pose[3];
float odom_vel[3];

/*
 * Declaration for setup flag.
 */
bool setup_end        = false;

uint32_t millis(void)
{
    return HAL_GetTick();
}

float constrain(float x, float low_val, float high_val)
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

void ros_setup(void)
{
    nh.initNode();                      /*!< Init ROS node handle */

    nh.subscribe(cmd_vel_sub);          /*!< Subscribe "cmd_vel" topic to get motor cmd */
    nh.subscribe(reset_sub);            /*!< Subscribe "reset" topic */

    nh.advertise(imu_pub);              /*!< Register the publisher to "imu" topic */
    nh.advertise(cmd_vel_motor_pub);    /*!< Register the publisher to "cmd_vel_motor" topic */
    nh.advertise(odom_pub);             /*!< Register the publisher to "odom" topic */
    nh.advertise(joint_states_pub);     /*!< Register the publisher to "joint_states" topic */

    tf_broadcaster.init(nh);            /*!< Init TransformBroadcaster */
    initOdom();                         /*!< Init odometry value */
    initJointStates();                  /*!< Init joint state */

    prev_update_time = millis();        /*!< Update time */
    setup_end = true;                   /*!< Flag for setup completed */
}

void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    /* Get goal velocity */
    goal_velocity_from_cmd[LINEAR] = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

    /* Constrain velocity */
    goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    /* Update time */
    tTime[CONTROL_MOTOR_TIMEOUT_TIME_INDEX] = millis();
}

void resetCallback(const std_msgs::Empty &reset_msg)
{
    char log_msg[50];

    (void)(reset_msg);

    sprintf(log_msg, "Start Calibration of Gyro");
    nh.loginfo(log_msg);

    initOdom();

    sprintf(log_msg, "Reset Odometry");
    nh.loginfo(log_msg);
}

void publishCmdVelFromMotorMsg(void)
{
    /* Get motor velocity */
    cmd_vel_motor_msg.linear.x = goal_velocity_from_motor[LINEAR];
    cmd_vel_motor_msg.angular.z = goal_velocity_from_motor[ANGULAR];

    /* Publish veloctiy to "cmd_vel_motor" topic */
    cmd_vel_motor_pub.publish(&cmd_vel_motor_msg);
}

void publishImuMsg(void)
{
    /* Get IMU data (accelerometer, gyroscope, quaternion and variance ) */
    imu_msg = getIMU();

    imu_msg.header.stamp = rosNow();
    imu_msg.header.frame_id = imu_frame_id;

    /* Publish IMU messages */
    imu_pub.publish(&imu_msg);
}

void publishDriveInformation(void)
{
    /* Update time */
    unsigned long time_now = millis();
    unsigned long step_time = time_now - prev_update_time;
    prev_update_time = time_now;
    ros::Time stamp_now = rosNow();

    /* Calculate odometry */
    calcOdometry((float)(step_time * 0.001f));

    /* Publish odometry message */
    updateOdometry();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    /* Publish TF message */
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    /* Publish jointStates message */
    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
}

void updateVariable(bool isConnected)
{
    static bool variable_flag = false;

    if (isConnected)
    {
        if (variable_flag == false)
        {
//            initIMU();
            initOdom();

            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}

void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
    int32_t current_tick = 0;

    if (init_encoder)
    {
        for (int index = 0; index < WHEEL_NUM; index++)
        {
            last_diff_tick[index] = 0;
            last_rad[index]       = 0.0f;

            last_velocity[index]  = 0.0f;
        }

        init_encoder = false;
        return;
    }

    last_diff_tick[LEFT] = left_tick;
    last_rad[LEFT]       += TICK2RAD * (float)last_diff_tick[LEFT];

    last_diff_tick[RIGHT] = right_tick;
    last_rad[RIGHT]       += TICK2RAD * (float)last_diff_tick[RIGHT];
}

void updateTime(void)
{
    current_offset = millis();
    current_time = nh.now();
}

ros::Time rosNow(void)
{
    return nh.now();
}

ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
    uint32_t sec, nsec;

    sec  = _micros / 1000 + t.sec;
    nsec = _micros % 1000000000 + t.nsec;

    return ros::Time(sec, nsec);
}

void updateOdometry(void)
{
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id  = odom_child_frame_id;

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];
}

void updateJointStates(void)
{
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

    joint_states_pos[LEFT]  = last_rad[LEFT];
    joint_states_pos[RIGHT] = last_rad[RIGHT];

    joint_states_vel[LEFT]  = last_velocity[LEFT];
    joint_states_vel[RIGHT] = last_velocity[RIGHT];

    joint_states.position = (double *)joint_states_pos;
    joint_states.velocity = (double *)joint_states_vel;
}

void updateJoint(void)
{

}

void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

void updateGyroCali(bool isConnected)
{
    static bool isEnded = false;
    char log_msg[50];

    (void)(isConnected);

    if (nh.connected())
    {
        if (isEnded == false)
        {
            sprintf(log_msg, "Start Calibration of Gyro");
            nh.loginfo(log_msg);

            //calibrationGyro();

            sprintf(log_msg, "Calibration End");
            nh.loginfo(log_msg);

            isEnded = true;
        }
    }
    else
    {
        isEnded = false;
    }
}

void updateGoalVelocity(void)
{
    goal_velocity[LINEAR]  = goal_velocity_from_cmd[LINEAR];
    goal_velocity[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}

void updateTFPrefix(bool isConnected)
{
    static bool isChecked = false;
    char log_msg[50];

    if (isConnected)
    {
        if (isChecked == false)
        {
            nh.getParam("~tf_prefix", &get_tf_prefix);

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
            nh.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
            nh.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
            nh.loginfo(log_msg);

            isChecked = true;
        }
    }
    else
    {
        isChecked = false;
    }
}

void initOdom(void)
{
    init_encoder = true;

    for (int index = 0; index < 3; index++)
    {
        odom_pose[index] = 0.0;
        odom_vel[index]  = 0.0;
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

void initJointStates(void)
{
    static char *joint_states_name[] = {(char*)"wheel_left_joint", (char*)"wheel_right_joint"};

    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name            = joint_states_name;

    joint_states.name_length     = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length   = WHEEL_NUM;
}

bool calcOdometry(float diff_time)
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

    wheel_l = TICK2RAD * (float)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (float)last_diff_tick[RIGHT];

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
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // compute odometric instantaneouse velocity

    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT]  = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;

    return true;
}

void sendLogMsg(void)
{
    static bool log_flag = false;

    if (nh.connected())
    {
        if (log_flag == false)
        {
            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            sprintf(log_msg, "Connected to openSTM32-Board");
            nh.loginfo(log_msg);

            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            log_flag = true;
        }
    }
    else
    {
        log_flag = false;
    }
}

void waitForSerialLink(bool isConnected)
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

sensor_msgs::Imu getIMU(void)
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

void controlMotor(float *goal_vel)
{
    float wheel_velocity_cmd[2];

    float lin_vel = goal_vel[LEFT];
    float ang_vel = goal_vel[RIGHT];

    wheel_velocity_cmd[LEFT]  = lin_vel - (ang_vel * WHEEL_SEPARATION / 2);
    wheel_velocity_cmd[RIGHT] = lin_vel + (ang_vel * WHEEL_SEPARATION / 2);

    wheel_velocity_cmd[LEFT]  = constrain(wheel_velocity_cmd[LEFT], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
    wheel_velocity_cmd[RIGHT] = constrain(wheel_velocity_cmd[RIGHT], MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);

    periph_motor_left_set_speed(wheel_velocity_cmd[LEFT]);
    periph_motor_right_set_speed(wheel_velocity_cmd[RIGHT]);
}

void getMotorSpeed(float *vel)
{
    goal_velocity_from_motor[LINEAR] = goal_velocity_from_cmd[LINEAR];
    goal_velocity_from_motor[ANGULAR] = goal_velocity_from_cmd[ANGULAR];
}
