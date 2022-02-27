#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "op3_walking_module_msgs/WalkingParam.h"

double x_move_amplitude=0;
double y_move_amplitude=0;
double angle_move_amplitude=0;

double position_x = 0.0;
double position_y = 0.0;
double orientation_yaw = 0.0;

int count = 0;
int step_num = 0;

bool walking_is_start = false;

void walking_paramCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg)
{
    x_move_amplitude = msg->x_move_amplitude;
    y_move_amplitude = msg->y_move_amplitude;
    angle_move_amplitude = msg->angle_move_amplitude;
}

void walkingCommandCallback(const std_msgs::String::ConstPtr& msg)
{
//  ROS_INFO("cc");
  if(msg->data == "start"){
    walking_is_start = true;
//    ROS_INFO("aa");
  }
  else if(msg->data == "stop"){
    walking_is_start = false;
//    ROS_INFO("bb");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_odom_node");
  ros::NodeHandle nh;

  ros::Publisher x_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_x", 1000);
  ros::Publisher y_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_y", 1000);
  ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/orientation_yaw", 1000);
  ros::Publisher step_num_pub = nh.advertise<std_msgs::Int32>("kubot_Pose/step_num", 1000);

  ros::Subscriber walking_command_sub = nh.subscribe("/robotis/walking/command", 0, walkingCommandCallback);
  ros::Subscriber sub = nh.subscribe("/robotis/walking/set_params", 1000, walking_paramCallback);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    std_msgs::Float64 x_msg;
    std_msgs::Float64 y_msg;
    std_msgs::Float64 yaw_msg;
    std_msgs::Int32 step_num_msg;

   // msg.data = "hello world";
    x_msg.data = position_x;
    y_msg.data = position_y;
    yaw_msg.data = 180*orientation_yaw/3.1415926535;
    step_num_msg.data = step_num;

    if((count >=30)&&(walking_is_start == true)){ //600ms
      step_num++;
      position_x += x_move_amplitude;
      position_y += y_move_amplitude;
      orientation_yaw += angle_move_amplitude;
      count = 0;
    }

    x_pub.publish(x_msg);
    y_pub.publish(y_msg);
    yaw_pub.publish(yaw_msg);
    step_num_pub.publish(step_num_msg);

    ros::spinOnce();
    count++;

    loop_rate.sleep();

  }

  return 0;
}
