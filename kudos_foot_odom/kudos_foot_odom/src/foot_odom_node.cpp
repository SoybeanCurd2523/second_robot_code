#include "foot_odom_node.h"

double x_move_amplitude=0;
double y_move_amplitude=0;
double angle_move_amplitude=0;

double position_x = 0.0;
double position_y = 0.0;
double orientation_yaw = 0.0;
int step_num = 0;

int i = 1;
int j = 1;
int rotate_count = 0;

bool walking_is_start = false;

int walkingstate = enable_odom;

double error = 0;
void walkingParamCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr& msg)
{
    x_move_amplitude = msg->x_move_amplitude;
    if(x_move_amplitude < 0)
      x_move_amplitude *= -1;
    y_move_amplitude = msg->y_move_amplitude;
    angle_move_amplitude = msg->angle_move_amplitude;
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

double errorfunction(double position_y, double orientation_yaw){
  position_x = position_y / tan(orientation_yaw);
  return position_x;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_odom_node"); // node name 
  ros::NodeHandle nh;

/*
  ros::Publisher x_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_x", 1000); //topic name
  ros::Publisher y_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/position_y", 1000);
  ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("kubot_Pose/orientation_yaw", 1000);
  ros::Publisher step_num_pub = nh.advertise<std_msgs::Int32>("kubot_Pose/step_num", 1000);
*/
  ros::Subscriber walking_command_sub = nh.subscribe("/robotis/walking/command", 0, walkingCommandCallback);
  ros::Subscriber sub = nh.subscribe("/robotis/walking/set_params", 1000, walkingParamCallback);
  
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std_msgs::Float64 x_msg;
    std_msgs::Float64 y_msg;
    std_msgs::Float64 yaw_msg;
    std_msgs::Int32 step_num_msg;

    step_num++;
    
    //define walkingstate
    if( (x_move_amplitude != 0) && (y_move_amplitude == 0) && (angle_move_amplitude == 0) )
      walkingstate = 0;
    else if( (x_move_amplitude != 0) && (y_move_amplitude != 0) && (angle_move_amplitude !=0) )
      walkingstate = 1;
    else if( (x_move_amplitude != 0) && (y_move_amplitude == 0) && (angle_move_amplitude != 0) )
      walkingstate = 2;
    else  
      walkingstate = 3;

    //calculation rotate count
    orientation_yaw = orientation_yaw + angle_move_amplitude
    if(orientation_yaw >= 2 * i * M_PI)
    {
      i++;
      rotate_count++;
      ROS_INFO(" rotate [%d] times", rotate_count);
    }

    // odometry
    switch(walkingstate)
    {
      case go_straight:
        ROS_INFO("go straight");

        if(j<=3)
        {
          position_x = position_x + (0.002/3);
          j++
        }
        else
          position_x = position_x + x_move_amplitude * cos(orientation_yaw) * 1.25; 

        position_y += position_y;
        break;
        
      case rotate_with_ball: 
        ROS_INFO("rotate with ball");
        position_x = position_x + x_move_amplitude * cos(orientation_yaw);
        position_y = position_y + x_move_amplitude * sin(orientation_yaw);
        error = position_x - errorfunction(position_y, orientation_yaw);
        ROS_INFO("error : %lf", error);
        break;

      case rotate_without_ball: 
        ROS_INFO("rotate without ball");
        position_x = position_x + x_move_amplitude * cos(orientation_yaw);
        position_y += position_y;
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

    x_pub.publish(x_msg);
    y_pub.publish(y_msg);
    yaw_pub.publish(yaw_msg);

    ROS_INFO("position_x : [%lf] centi meter", position_x * 1000);
    ROS_INFO("position_y : [%lf] centi meter", position_y * 1000);
    ROS_INFO("orientation_yaw : [%lf] degree", orientation_yaw * RAD2DEG);

    step_num_pub.publish(step_num_msg);

    ros::spinOnce();
    count++;

    loop_rate.sleep();

  }
  ros::spin(); 

  return 0;
}
