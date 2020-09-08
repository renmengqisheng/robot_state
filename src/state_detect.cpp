#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <stdio.h>

const float pi = 3.1415926;

const float roll_threshold = 30;
const float pitch_threshold = 30;
const float yaw_threshold = 30;

const float angular_velocity_x_threshold = 0.05;
const float angular_velocity_y_threshold = 0.05;
const float angular_velocity_z_threshold = 0.05;

double roll = 0.0, pitch = 0.0, yaw = 0.0;
double angular_velocity[3] = {0};

bool is_motion(double velocity[3])
{
    if(fabs(velocity[0]) < angular_velocity_x_threshold && fabs(velocity[1]) < angular_velocity_y_threshold && fabs(velocity[2]) < angular_velocity_z_threshold)
    {
        return false;
    }
    else
    {
        return true;
    }
        
}


void IMU_read(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // debug
    // printf("x: %f, y: %f, z: %f, w: %f\n\n", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // 进行转换
    roll = roll / pi * 180;
    pitch = pitch / pi * 180;
    yaw = yaw / pi * 180;

    angular_velocity[0] = msg->angular_velocity.x;
    angular_velocity[1] = msg->angular_velocity.y;
    angular_velocity[2] = msg->angular_velocity.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_detect");

    ros::NodeHandle node;
    ros::Subscriber IMU = node.subscribe("imu", 1, IMU_read);
    ros::Publisher state_pub = node.advertise<std_msgs::String>("robotis/base/ini_pose", 20);

    ros::Rate r(20);
    while(ros::ok())
    {
        std_msgs::String state;
        ros::spinOnce();

        // debug
        // printf("roll: %f, pitch: %f, yaw: %f\n\n", roll, pitch, yaw);

        if(is_motion(angular_velocity))
        {
            state.data = "normal";  // 正常状态
        }
        else
        {
            if(roll > -roll_threshold && roll < roll_threshold && pitch > -pitch_threshold && pitch < pitch_threshold)
            {
                state.data = "normal";  // 正常状态
            }
            else if(roll > -roll_threshold && roll < roll_threshold && pitch > pitch_threshold)  // 正前方
            {
                state.data = "up_front";
            }
            else if(roll < -roll_threshold  && pitch > pitch_threshold)  // 左前方
            {
                state.data = "up_left_front";
            }
            else if(roll > roll_threshold  && pitch > pitch_threshold) // 右前方
            {
                state.data = "up_right_front";
            }
            else if(roll < -roll_threshold && pitch > -pitch_threshold && pitch < pitch_threshold)  // 正左方
            {
                state.data = "up_left";
            }
            else if(roll > roll_threshold && pitch > -pitch_threshold && pitch < pitch_threshold)  // 正右方
            {
                state.data = "up_right";
            }
            else if(roll > -roll_threshold && roll < roll_threshold && pitch < -pitch_threshold) // 正后方
            {
                state.data = "up_back";
            }
            else if(roll < -roll_threshold && pitch < -pitch_threshold) // 左后方
            {
                state.data = "up_left_back";
            }
            else if(roll > roll_threshold && pitch < -pitch_threshold) // 右后方
            {
                state.data = "up_right_back";
            }
        }

        state_pub.publish(state);

        r.sleep();
    }

    return 0;
}