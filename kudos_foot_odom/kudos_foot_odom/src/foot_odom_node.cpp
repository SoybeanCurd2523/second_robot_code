// #include "foot_odom_node.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "math.h"

#define RAD2DEG 180/M_PI

double x_move_amplitude = 0.0;
double y_move_amplitude = 0.0;
double angle_move_amplitude = 0.0;

double position_x = 0.0;
double position_y = 0.0;
double orientation_yaw = 0.0;
double period_time = 0.0;

int i = 1;
int j = 1;
int step_num = 0;
int rotate_count = 0;

bool walking_is_start = false;

const int go_straight = 0;
const int rotate_without_ball = 1;
const int enable_odom = 2;

int walkingstate = enable_odom;
double error = 0.0;

void walkingParamCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg)
{
    x_move_amplitude = msg->x_move_amplitude;
    if(x_move_amplitude < 0)
      x_move_amplitude *= -1;
    y_move_amplitude = msg->y_move_amplitude;
    angle_move_amplitude = msg->angle_move_amplitude;
    period_time = msg->period_time;
}

void walkingCommandCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "start")
  {
    walking_is_start = true;
  }
  else if(msg->data == "stop")
  {
    walking_is_start = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_odom_node"); // node name 
  ros::NodeHandle nh;

  ros::Publisher x_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_x", 1000); //topic name
  ros::Publisher y_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_y", 1000);
  ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/orientation_yaw", 1000);
  ros::Publisher step_num_pub = nh.advertise<std_msgs::Int32>("kubot_Pose/step_num", 1000);
  ros::Publisher period_time_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/period_time", 1000);

  ros::Subscriber walking_command_sub = nh.subscribe("/robotis/walking/command", 0, walkingCommandCallback);
  ros::Subscriber sub = nh.subscribe("/robotis/walking/set_params", 1000, walkingParamCallback);
  
  // period_time : 600ms = 0.6s
  // 1/period_time : 1/0.6 = 1.667Hz
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::Float64 x_msg;
    std_msgs::Float64 y_msg;
    std_msgs::Float64 yaw_msg;
    std_msgs::Int32 step_num_msg;
    std_msgs::Float64 period_time_msg;

    step_num++;
    
    //define walkingstate --> relative angle
    if((x_move_amplitude != 0) && (y_move_amplitude == 0) && (angle_move_amplitude == 0))
      walkingstate = go_straight;
    else if
      ((x_move_amplitude != 0) && (y_move_amplitude == 0) && (angle_move_amplitude != 0))
      walkingstate = rotate_without_ball;
    else  
      walkingstate = enable_odom;
    
    //calculation rotate count
    orientation_yaw = orientation_yaw + angle_move_amplitude;
    if(orientation_yaw >= 2 * i * M_PI)
    {
      i++;
      rotate_count++;
      ROS_INFO("rotate [%d] times", rotate_count);
    }

    // odometry
    switch(walkingstate)
    {
      case go_straight:
        ROS_INFO("go straight");

          if(j<=5)
        {
          position_x = position_x + (0.004/5); // during first 5 step
          position_y += position_y;
          j++;
          error = abs(position_x - step_num * 0.004); // 1step -> 0.004m
        }
        else
        {
          position_x = position_x + x_move_amplitude * cos(orientation_yaw) * 1.25; // correction value 
          position_y = position_y + x_move_amplitude * sin(orientation_yaw) * 5;
        }
      
        ROS_INFO("current_x_error : [%lf] mili meter : %lf", error *10000);
        
        break;

      case rotate_without_ball: 
        ROS_INFO("rotate without ball");
        position_x = position_x + x_move_amplitude * cos(orientation_yaw);
        position_y = position_y + x_move_amplitude * sin(orientation_yaw);

        error = abs(orientation_yaw - step_num * 0.348888); // 18step -> 1rotate
        ROS_INFO("current_yaw_error : %lf", error);
        
        break;

      case enable_odom:
        ROS_INFO("enable odom");
        break;

      default:
        break;
    }

    x_msg.data = position_x;
    y_msg.data = position_y;
    yaw_msg.data = orientation_yaw * RAD2DEG;
    step_num_msg.data = step_num;
    period_time_msg.data = period_time;

    x_pub.publish(x_msg);
    y_pub.publish(y_msg);
    yaw_pub.publish(yaw_msg);
    step_num_pub.publish(step_num_msg);
    period_time_pub.publish(period_time_msg);

    ROS_INFO("position_x : [%lf] centi meter", position_x * 1000);
    ROS_INFO("position_y : [%lf] centi meter", position_y * 1000);
    ROS_INFO("orientation_yaw : [%lf] degree", orientation_yaw * RAD2DEG);
    ROS_INFO("=================================");

    ros::spinOnce();

    loop_rate.sleep();

  }
  ros::spin(); 

  return 0;
}
//}
