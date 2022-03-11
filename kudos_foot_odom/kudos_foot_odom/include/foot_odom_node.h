#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "math.h"

#define RAD2DEG 180/M_PI

class foot_odom{
public:
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

  void walkingParamCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg);
  void walkingCommandCallback(const std_msgs::String::ConstPtr& msg);
  // void walkingStateCallback(const std_msgs::String::ConstPtr& msg);

protected:
  ros::NodeHandle nh;

  ros::Rate loop_rate;

  ros::Publisher x_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_x", 1000); //topic name
  ros::Publisher y_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_y", 1000);
  ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/orientation_yaw", 1000);
  ros::Publisher step_num_pub = nh.advertise<std_msgs::Int32>("kubot_Pose/step_num", 1000);

  ros::Subscriber walking_command_sub = nh.subscribe("/robotis/walking/command", 0, walkingCommandCallback);
  ros::Subscriber sub = nh.subscribe("/robotis/walking/set_params", 1000, walkingParamCallback);
  // ros::Subscriber standup_state_sub = nh.subscribe("/robotis/walking/standup", 1000, walkingStateCallback); 
};