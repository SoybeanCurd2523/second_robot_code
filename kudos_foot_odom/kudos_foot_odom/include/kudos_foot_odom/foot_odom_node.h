#ifndef FOOT_ODOM_NODE_H
#define FOOT_ODOM_NODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "math.h"

#define RAD2DEG 180/M_PI

namspace robotis_op
{

class foot_odom_node{
public:
  foot_odom_node();
  ~foot_odom_node();

  double x_move_amplitude;
  double y_move_amplitude;
  double angle_move_amplitude;

  double position_x;
  double position_y;
  double orientation_yaw;
  double period_time;

  int i;
  int j;
  int rotate_count;
  int step_num;

  bool walking_is_start;
  int walkingstate;
  double error;
  void walkingParamCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg);
  void walkingCommandCallback(const std_msgs::String::ConstPtr& msg);

  double errorfunction(double position_y, double orientation_yaw)

protected:
  enum
  {
    go_straight= 0,
    rotate_without_ball = 1,
    enable_odom = 2,
  };

  ros::NodeHandle nh;

  ros::Rate loop_rate;

  ros::Publisher x_pub;
  ros::Publisher y_pub;
  ros::Publisher yaw_pub;
  ros::Publisher step_num_pub;
  ros::Publisher period_time_pub;

  ros::Subscriber walking_command_sub;
  ros::Subscriber sub;
};
}

#endif /* FOOT_ODOM_NODE_H */