#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <stdio.h>

const float pi = 3.1415926;

const float roll_threshold = 10;
const float pitch_threshold = 10;
const float yaw_threshold = 10;

double roll = 0.0, pitch = 0.0, yaw = 0.0;

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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_detect");

    ros::NodeHandle node;
    ros::Subscriber IMU = node.subscribe("imu", 1, IMU_read);
    ros::Publisher state_pub = node.advertise<std_msgs::String>("state", 20);

    ros::Rate r(20);
    while(ros::ok())
    {
        std_msgs::String state;
        ros::spinOnce();

        // debug
        // printf("roll: %f, pitch: %f, yaw: %f\n\n", roll, pitch, yaw);

        if(roll > -roll_threshold && roll < roll_threshold && pitch > -pitch_threshold && pitch < pitch_threshold)
        {
            state.data = "NORMAL";  // 正常状态
        }
        else if(roll > -roll_threshold && roll < roll_threshold && pitch > pitch_threshold)  // 正前方
        {
            state.data = "FRONT";
        }
        else if(roll < -roll_threshold  && pitch > pitch_threshold)  // 左前方
        {
            state.data = "LEFT_FRONT";
        }
        else if(roll > roll_threshold  && pitch > pitch_threshold) // 右前方
        {
            state.data = "RIGHT_FRONT";
        }
        else if(roll < -roll_threshold && pitch > -pitch_threshold && pitch < pitch_threshold)  // 正左方
        {
            state.data = "LEFT";
        }
        else if(roll > roll_threshold && pitch > -pitch_threshold && pitch < pitch_threshold)  // 正右方
        {
            state.data = "RIGHT";
        }
        else if(roll > -roll_threshold && roll < roll_threshold && pitch < -pitch_threshold) // 正后方
        {
            state.data = "BACK";
        }
        else if(roll < -roll_threshold && pitch < -pitch_threshold) // 左后方
        {
            state.data = "LEFT_BACK";
        }
        else if(roll > roll_threshold && pitch < -pitch_threshold) // 右后方
        {
            state.data = "RIGHT_BACK";
        }

        state_pub.publish(state);

        r.sleep();
    }

    return 0;
}