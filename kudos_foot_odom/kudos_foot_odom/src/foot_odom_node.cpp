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

int walkingstate = 0;

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
  if(msg->data == "start"){
    walking_is_start = true;angle_move_amplitude 
  }
  else if(msg->data == "stop"){
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

  ros::Subscriber walking_command_sub = nh.subscribe("/robotis/walking/command", 0, walkingCommandCallback);
  ros::Subscriber sub = nh.subscribe("/robotis/walking/set_params", 1000, walkingParamCallback);
  
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std_msgs::Float64 x_msg;
    std_msgs::Float64 y_msg;
    std_msgs::Float64 yaw_msg;
    std_msgs::Int32 step_num_msg;

   // double orientation_yaw = 0.0;
    step_num++;
    
    if(orientation_yaw == 0)
      walkingstate = 0;
    else
      walkingstate = 1;


    orientation_yaw = orientation_yaw + angle_move_amplitude
    if(orientation_yaw >= 2 * i * M_PI)
    {
      i++;
      rotate_count++;
      ROS_INFO(" rotate [%d] times", rotate_count);
    }



    switch(walkingstate)
    {
      case 0: //직진  y,yaw = 0
        ROS_INFO("go straight");
        position_x = position_x + x_move_amplitude * cos(orientation_yaw) * 1.25; //보정계수
        position_y = 0;
        break;
      
      case 1: //공보고 회전 
        ROS_INFO("find ball");
        position_x = position_x + x_move_amplitude * cos(orientation_yaw);
        position_y = position_y + x_move_amplitude * sin(orientation_yaw);
        break;



      case 2: //공안볼때 자전 y=0
        position_y = 0;
        position_x = position_x + x_move_amplitude * cos(orientation_yaw);
        break;


      defalut: //기타 - 넘어졌을때나 킥할때?
        break;
    }
    // if((j<=3) && (x_move_amplitude != 0))
    //   position_x = position_x + (0.002/3)
    // else
    //   position_x = position_x + x_move_amplitude * cos(orientation_yaw) * 1.25; //보정계수
    
    // if(orientation_yaw != 0 ,22344544) //회전할 때는 y, yaw -> x,y position 구해서 비교해보기
    //   position_x = position_y / tan(orientation_yaw)


// 넘어졌을 때 킥할때 버튼눌렀을 때 누적 안하기
// 회전할 때는 y, yaw -> x,y position 구해서 비교해보기
// 로봇 주기 가져오기?
// 로봇 pc 발열이 심함
// 킥
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
