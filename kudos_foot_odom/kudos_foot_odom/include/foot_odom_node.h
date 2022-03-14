#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "math.h"

#define RAD2DEG 180/M_PI

class foot_odom{
public:
  enum
  {
    go_straight= 0,
    rotate_with_ball = 1,
    rotate_without_ball = 2,
    enable_odom = 3,
  };

  foot_odom();
  ~foot_odom();

  double x_move_amplitude;
  double y_move_amplitude;
  double angle_move_amplitude;

  double position_x;
  double position_y;
  double orientation_yaw;

  int step_num;

  int i;
  int j;
  int rotate_count;

  bool walking_is_start;
  // bool standup_state; 
  double error;
  void walkingParamCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg);
  void walkingCommandCallback(const std_msgs::String::ConstPtr& msg);

  double errorfunction(double position_y, double orientation_yaw)
  // void walkingStateCallback(const std_msgs::String::ConstPtr& msg);

protected:
  int walkingstate;
  ros::NodeHandle nh;

  ros::Rate loop_rate;

  ros::Publisher x_pub;
  ros::Publisher y_pub;
  ros::Publisher yaw_pub];
  ros::Publisher step_num_pub;

  ros::Subscriber walking_command_sub;
  ros::Subscriber sub;
  // ros::Subscriber standup_state_sub = nh.subscribe("/robotis/walking/standup", 1000, walkingStateCallback); 
};