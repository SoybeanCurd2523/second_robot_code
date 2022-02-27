#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

void XdataCallback(const std_msgs::Float64::ConstPtr& x_msg)
{
    ROS_INFO("Received: [%f]", x_msg->data);
}

void YdataCallback(const std_msgs::Float64::ConstPtr& y_msg)
{
    ROS_INFO("Received: [%f]", y_msg->data);
}

void YawdataCallback(const std_msgs::Float64::ConstPtr& yaw_msg)
{
    ROS_INFO("Received: [%f]", yaw_msg->data);
}

void StepdataCallback(const std_msgs::Int32::ConstPtr& step_num_msg)
{
    ROS_INFO("Received: [%d]", step_num_msg->data);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "foot_localization");
    ros::NodeHandle nh;

    ros::Subscriber x_sub = nh.subscribe("kubot_Pose/position_x", 1000, XdataCallback);
    ros::Subscriber y_sub = nh.subscribe("kubot_Pose/position_y", 1000, YdatalCallback);
    ros::Subscriber yaw_sub = nh.subscribe("kubot_Pose/orientation", 1000, YawdataCallback);
    ros::Subscriber step_num_sub = nh.subscribe("kubot_Pose/step_num", 1000, StepdataCallback);
    
    ros::spin();

    return 0;
}