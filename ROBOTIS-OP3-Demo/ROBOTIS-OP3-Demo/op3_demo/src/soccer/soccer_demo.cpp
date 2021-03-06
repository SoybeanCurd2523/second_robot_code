/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#include "op3_demo/soccer_demo.h"

int turn_around = 0;
int count_1 = 0;

namespace robotis_op
{

SoccerDemo::SoccerDemo()
  : FALL_FORWARD_LIMIT(70),
    FALL_BACK_LIMIT(-60),
    SPIN_RATE(30),
    DEBUG_PRINT(false),
    wait_count_(0),
    on_following_ball_(false),
    on_tracking_ball_(false),
    restart_soccer_(false),
    start_following_(false),
    stop_following_(false),
    stop_fallen_check_(false),
    robot_status_(Waited),
    stand_state_(Stand),
    tracking_status_(BallTracker::Waiting),
    present_pitch_(0)
{
  //init ros
  enable_ = false;

  ros::NodeHandle nh(ros::this_node::getName());

  std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/gui_config.yaml";
  std::string path = nh.param<std::string>("demo_config", default_path);
  parseJointNameFromYaml(path);

  boost::thread queue_thread = boost::thread(boost::bind(&SoccerDemo::callbackThread, this));
  boost::thread process_thread = boost::thread(boost::bind(&SoccerDemo::processThread, this));
  boost::thread tracking_thread = boost::thread(boost::bind(&SoccerDemo::trackingThread, this));

  kudos_vision_op3_local_mode_msg_ = nh.advertise<op3_ball_detector::kudos_vision_op3_local_mode>("/kudos_vision_op3_local_mode", 0);
  gcm_sub_ = nh.subscribe("/kudos_vision_gcm", 1, &SoccerDemo::gameControllerMessageCallback, this);
  mcl2_sub_ = nh.subscribe("/kudos_vision_mcl2_local_result", 1, &SoccerDemo::mcl2_local_result_MessageCallback, this);
  set_walking_command_pub_ = nh.advertise<std_msgs::String>("/robotis/walking/command", 0);
  pre_gcm_game_state = 999;

  is_grass_ = nh.param<bool>("grass_demo", false);

  //sensor_yaw_sub_ = nh.subscribe("/kubot/sensor_yaw", 1000, &SoccerDemo::sensor_yaw_callback, this);

  // imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &SoccerDemo::imuDataCallback, this); //pjh
}

SoccerDemo::~SoccerDemo()
{

}
void SoccerDemo::gameControllerMessageCallback(const op3_ball_detector::kudos_vision_gcm::ConstPtr &msg)
{
  // kudos_vision_gcm.msg   path : /home/nvidia/catkin_ws/src/ROBOTIS-OP3-Demo/op3_ball_detector/msg/kudos_vision_gcm.msg
  int tmp_state = msg-> main_game_state;
  Is_penalty = msg -> Is_penalty;
  Secondary_remaining = msg-> Secondary_remaining;
  gcm_vision_go_local = msg -> vision_go_local;
  start_point_x = msg->fake_start_point_x;
  start_point_y = msg->fake_start_point_y;
  start_point_orien = msg->fake_start_point_orien;

  if(tmp_state == 0)
  gcm_game_state = 0;
  else if(tmp_state == 1)
  gcm_game_state = 1;
  else if(tmp_state == 2)
  gcm_game_state = 2;
  else if(tmp_state == 3 && Is_penalty != 1)
  gcm_game_state = 3;
  else if(tmp_state == 4)
  gcm_game_state = 4;
  else if(tmp_state == 3 && Is_penalty == 1 && (Secondary_remaining ==0))
  gcm_game_state = 5;
  else if(tmp_state == 3 && Is_penalty == 1 && (Secondary_remaining > 0) && (Secondary_remaining <= 30))
  gcm_game_state = 6;

  printf("bh_init_setting:%d\n", bh_init_setting);
  printf("gcm_game_state:%d\n", gcm_game_state);
  printf("Is_penalty:%d\n", Is_penalty);
  printf("Secondary_remaining:%d\n", Secondary_remaining);

}

void SoccerDemo::mcl2_local_result_MessageCallback(const op3_ball_detector::kudos_vision_mcl2_local_result::ConstPtr &msg)
{
  local_result_x = msg->local_result_x;
  local_result_y = msg->local_result_y;
  local_result_orien = msg->local_result_orien;

  printf("local_result_x:%lf && local_result_y:%lf && local_result_orien:%lf \n", local_result_x, local_result_y, local_result_orien);
}

void SoccerDemo::sensor_yaw_callback(const std_msgs::Float64::ConstPtr &msg)
{
  kudos_imu_yaw = msg->data;
}

void SoccerDemo::setDemoEnable()
{
  enable_ = true;

  startSoccerMode();
}

void SoccerDemo::setDemoDisable()
{
  // handle disable procedure
  ball_tracker_.stopTracking();
  ball_follower_.stopFollowing();

  enable_ = false;
  wait_count_ = 0;
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  restart_soccer_ = false;
  start_following_ = false;
  stop_following_ = false;
  stop_fallen_check_ = false;

  tracking_status_ = BallTracker::Waiting;
}
//

void SoccerDemo::gcm_mode_init()
{
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  stop_following_ = true;
}

void SoccerDemo::gcm_mode_set()
{
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  stop_following_ = true;
}

void SoccerDemo::gcm_mode_play()
{
  on_following_ball_ = true;
  on_tracking_ball_ = true;
  start_following_ = true;
}

void SoccerDemo::process()
{
if(enable_ == false)
  return;

// open_cr_module_.printfunction(); //pjh

if(pre_gcm_game_state != gcm_game_state)
{
  pre_gcm_game_state = gcm_game_state;
  enable_set_mode_trigger = true;
}
if(enable_set_mode_trigger == true)
{
  enable_set_mode_trigger = false;
  switch(gcm_game_state)
  {
    case 0: //initial
      gcm_mode_init();
      break;

    case 1: // Ready
      //ball_follower_.casemove_(); // HS
      count = 0;
      //ball_follower_.setWalkingParam(0.1, 0, 0, true);
      // for(count = 0; count < 200; count++)
      //   ball_follower_.case_nm("start");

      for(count=0; count < 1200; count++)
        ball_follower_.case_move("start");

      //for(count = 1700; count < 2000; count++)
      //  ball_follower_.case_nm("start");

      for(count= 0; count < 700; count++)
        ball_follower_.case_turn("start");
      //ball_follower_.setWalkingParam(0, 0, 0.5, true);

      ball_follower_.setWalkingCommand("stop");
      break;

    case 2: //set
      gcm_mode_set();
      break;

    case 3: // playing
      gcm_mode_play();
      break;

    case 4: //Finished
      gcm_mode_set();
      break;
    case 5: //penalty level_1
      gcm_mode_set();
      break;

    case 6: //penalty level_2
      gcm_mode_play();
      break;
    }
  }


  if (start_following_ == true)
  {
    ball_tracker_.startTracking();
    ball_follower_.startFollowing();
    start_following_ = false;

    wait_count_ = 1 * SPIN_RATE;
  }

  // check to stop
  if (stop_following_ == true)
  {
    ball_tracker_.stopTracking();
    ball_follower_.stopFollowing();
    stop_following_ = false;

    wait_count_ = 0;
  }

  if (wait_count_ <= 0)
  {
    // ball following
    if (on_following_ball_ == true)
    {
      switch(tracking_status_)
      {
      case BallTracker::Found:
        ball_follower_.processFollowing(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall(), 0.0);
        break;

      case BallTracker::NotFound:
        ball_follower_.waitFollowing();
        break;

      default:
        break;
      }
    }

    // check fallen states
    switch (stand_state_)
    {
    case Stand:
    {
      // check restart soccer
      if (restart_soccer_ == true)
      {
        restart_soccer_ = false;
        startSoccerMode();
        break;
      }

      // check states for kick
      int ball_position = ball_follower_.getBallPosition();
      bool in_range = ball_follower_.isBallInRange();

      if(in_range == true)
      {
        ball_follower_.stopFollowing();
        handleKick();
      }
      
      break;
    }
      // fallen state : Fallen_Forward, Fallen_Behind
    default:
    {
      ball_follower_.stopFollowing();
      handleFallen(stand_state_);
      break;
    }
    }
  }
  else
  {
    wait_count_ -= 1;
  }
}
//
void SoccerDemo::processThread()
{
  bool result = false;

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE); //30hz

  ball_tracker_.startTracking();

  //node loop
  while (ros::ok())
  {
    if (enable_ == true)
      process();

    //relax to fit output rate
    loop_rate.sleep();
  }
}

void SoccerDemo::callbackThread()
{
  ros::NodeHandle nh(ros::this_node::getName());

  // subscriber & publisher
  module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
  motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
  rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

  buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &SoccerDemo::buttonHandlerCallback, this);
  // open_cr_module.cpp   path : /home/nvidia/catkin_ws/src/ROBOTIS-OP3/open_cr_module/src/open_cr_module.cpp
  demo_command_sub_ = nh.subscribe("/robotis/demo_command", 1, &SoccerDemo::demoCommandCallback, this);
  imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &SoccerDemo::imuDataCallback, this);

  is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
  set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetJointModule>("/robotis/set_present_joint_ctrl_modules");

  test_pub_ = nh.advertise<std_msgs::String>("/debug_text", 0);

  while (nh.ok())
  {
    ros::spinOnce();
    usleep(1000);
  }
}
/*
void SoccerDemo::trackingThread()
{

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  ball_tracker_.startTracking();

  //node loop
  while (ros::ok())
  {

    if(enable_ == true && on_tracking_ball_ == true)
    {
      // ball tracking
      int tracking_status;

      tracking_status = ball_tracker_.processTracking();

      // set led
      switch(tracking_status)
      {
      case BallTracker::Found:
        if(tracking_status_ != tracking_status)
          setRGBLED(0x1F, 0x1F, 0x1F);
        break;

      case BallTracker::NotFound:
        if(tracking_status_ != tracking_status)
          setRGBLED(0, 0, 0);
        break;

      default:
        break;
      }

      if(tracking_status != tracking_status_)
        tracking_status_ = tracking_status;
    }
    //relax to fit output rate
    loop_rate.sleep();
  }
}
*/
void SoccerDemo::trackingThread()
{

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  //node loop
  while (ros::ok())
  {
    if(enable_ == true && on_tracking_ball_ == true)
    {
      // ball tracking
      int tracking_status;
      //tracking_status = ball_tracker_.processTracking();
      
      if(gcm_vision_go_local == -1 && gcm_vision_go_local_init_setting != gcm_vision_go_local)
      {
        gcm_vision_go_local_init_setting = gcm_vision_go_local;
        std_msgs::String _command_msg;
        _command_msg.data = "start";
        set_walking_command_pub_.publish(_command_msg);
        ball_follower_.startFollowing();
        start_following_ = true;
        on_following_ball_ = true;
        printf("start Walking\n");
      }

      else if(gcm_vision_go_local == 1 && gcm_vision_go_local_init_setting != gcm_vision_go_local)
      {
        gcm_vision_go_local_init_setting = gcm_vision_go_local;
        std_msgs::String _command_msg;
        _command_msg.data = "stop";
        set_walking_command_pub_.publish(_command_msg);
        ball_follower_.stopFollowing();
        start_following_ = false;
        on_following_ball_ = false;
        printf("stop Walking\n");
      }


      else if(gcm_vision_go_local == -1 && gcm_vision_go_local_init_setting == gcm_vision_go_local)
      {
        //To do
        //?????? ?????? ??????: ????????? ?????????
        //
        op3_ball_detector::kudos_vision_op3_local_mode kudos_vision_tmp_data;
        kudos_vision_tmp_data.op3_local_mode = false;
        kudos_vision_tmp_data.start_point_x = 0;
        kudos_vision_tmp_data.start_point_y = 0;
        kudos_vision_tmp_data.start_point_orien = 0;
        kudos_vision_op3_local_mode_msg_.publish(kudos_vision_tmp_data);
        tracking_status = ball_tracker_.processTracking();
      }
      else if(gcm_vision_go_local == 1 && gcm_vision_go_local_init_setting == gcm_vision_go_local)
      {
        //To do
        //?????? ?????? ??????: ????????? ?????????
        std_msgs::String _command_msg;
        _command_msg.data = "stop";
        set_walking_command_pub_.publish(_command_msg);
        ball_follower_.stopFollowing();
        start_following_ = false;
        on_following_ball_ = false;
        //
        op3_ball_detector::kudos_vision_op3_local_mode kudos_vision_tmp_data;
        kudos_vision_tmp_data.op3_local_mode = true;
        kudos_vision_tmp_data.start_point_x = start_point_x;
        kudos_vision_tmp_data.start_point_y = start_point_y;
        kudos_vision_tmp_data.start_point_orien = start_point_orien;
        kudos_vision_op3_local_mode_msg_.publish(kudos_vision_tmp_data);
        tracking_status = ball_tracker_.processTracking_hand_over_authority_to_python();
      }
      

      // set led
      switch(tracking_status)
      {
      case BallTracker::Found:
        if(tracking_status_ != tracking_status)
          setRGBLED(0x1F, 0x1F, 0x1F);
        break;

      case BallTracker::NotFound:
        if(tracking_status_ != tracking_status)
          setRGBLED(0, 0, 0);
        break;

      default:
        break;
      }

      if(tracking_status != tracking_status_)
        tracking_status_ = tracking_status;
    }
    //relax to fit output rate
    loop_rate.sleep();
  }
}



void SoccerDemo::setBodyModuleToDemo(const std::string &body_module, bool with_head_control)
{
  robotis_controller_msgs::JointCtrlModule control_msg;

  std::string head_module = "head_control_module";
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    // check whether joint name contains "head"
    if (joint_iter->second.find("head") != std::string::npos)
    {
      if (with_head_control == true)
      {
        control_msg.joint_name.push_back(joint_iter->second);
        control_msg.module_name.push_back(head_module);
      }
      else
        continue;
    }
    else
    {
      control_msg.joint_name.push_back(joint_iter->second);
      control_msg.module_name.push_back(body_module);
    }
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  callServiceSettingModule(control_msg);
  std::cout << "enable module of body : " << body_module << std::endl;
}

void SoccerDemo::setModuleToDemo(const std::string &module_name)
{
  if(enable_ == false)
    return;

  robotis_controller_msgs::JointCtrlModule control_msg;
  std::map<int, std::string>::iterator joint_iter;

  for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
  {
    control_msg.joint_name.push_back(joint_iter->second);
    control_msg.module_name.push_back(module_name);
  }

  // no control
  if (control_msg.joint_name.size() == 0)
    return;

  callServiceSettingModule(control_msg);
  std::cout << "enable module : " << module_name << std::endl;
}

void SoccerDemo::callServiceSettingModule(const robotis_controller_msgs::JointCtrlModule &modules)
{
  robotis_controller_msgs::SetJointModule set_joint_srv;
  set_joint_srv.request.joint_name = modules.joint_name;
  set_joint_srv.request.module_name = modules.module_name;

  if (set_joint_module_client_.call(set_joint_srv) == false)
  {
    ROS_ERROR("Failed to set moduletqtq");
    return;
  }

  return ;
}

void SoccerDemo::parseJointNameFromYaml(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load id_joint table yaml.");
    return;
  }

  // parse id_joint table
  YAML::Node _id_sub_node = doc["id_joint"];
  for (YAML::iterator _it = _id_sub_node.begin(); _it != _id_sub_node.end(); ++_it)
  {
    int _id;
    std::string _joint_name;

    _id = _it->first.as<int>();
    _joint_name = _it->second.as<std::string>();

    id_joint_table_[_id] = _joint_name;
    joint_id_table_[_joint_name] = _id;
  }
}

// joint id -> joint name
bool SoccerDemo::getJointNameFromID(const int &id, std::string &joint_name)
{
  std::map<int, std::string>::iterator _iter;

  _iter = id_joint_table_.find(id);
  if (_iter == id_joint_table_.end())
    return false;

  joint_name = _iter->second;
  return true;
}

// joint name -> joint id
bool SoccerDemo::getIDFromJointName(const std::string &joint_name, int &id)
{
  std::map<std::string, int>::iterator _iter;

  _iter = joint_id_table_.find(joint_name);
  if (_iter == joint_id_table_.end())
    return false;

  id = _iter->second;
  return true;
}

int SoccerDemo::getJointCount()
{
  return joint_id_table_.size();
}

bool SoccerDemo::isHeadJoint(const int &id)
{
  std::map<std::string, int>::iterator _iter;

  for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
  {
    if (_iter->first.find("head") != std::string::npos)
      return true;
  }

  return false;
}

void SoccerDemo::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    

    if (on_following_ball_ == true)
      stopSoccerMode();
    else
    {
      ros::Duration(30).sleep(); //pjh 

      startSoccerMode();

      ROS_INFO("play run1 on %d time", count_1 + 2);
      count_1++;

    }
  }
}

void SoccerDemo::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
{
  if (enable_ == false)
    return;

  if (msg->data == "start")
  {
    if (on_following_ball_ == true)
      stopSoccerMode();
    else
      startSoccerMode();
  }
  else if (msg->data == "stop")
  {
    stopSoccerMode();
  }
}

// check fallen states
void SoccerDemo::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (stop_fallen_check_ == true)
    return;

  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / M_PI);

  ROS_INFO_COND(DEBUG_PRINT, "Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

  double pitch = rpy_orientation.coeff(1, 0);

  double alpha = 0.4;
  if (present_pitch_ == 0)
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT)
    stand_state_ = Fallen_Forward;
  else if (present_pitch_ < FALL_BACK_LIMIT)
    stand_state_ = Fallen_Behind;
  else
    stand_state_ = Stand;
}

void SoccerDemo::startSoccerMode()
{
  setModuleToDemo("action_module");

  playMotion(WalkingReady);

  setBodyModuleToDemo("walking_module");

  ROS_INFO("Start Soccer Demo");
  on_following_ball_ = true;
  on_tracking_ball_ = true;
  start_following_ = true;
}

void SoccerDemo::stopSoccerMode()
{
  ROS_INFO("Stop Soccer Demo");
  on_following_ball_ = false;
  on_tracking_ball_ = false;
  stop_following_ = true;
}

void SoccerDemo::handleKick(int ball_position)
{
  usleep(1500 * 1000);

  ROS_INFO("kudos_imu_yaw::::%lf \n", kudos_imu_yaw);

  int jy = -1;

  if (jy == -1)
  {
    // change to motion module
    setModuleToDemo("action_module");

    if (handleFallen(stand_state_) == true || enable_ == false)
      return;

    
//    if ( (kudos_imu_yaw<-30) && (kudos_imu_yaw>-150) )
    // kick motion
    switch (ball_position)
    {
      case robotis_op::BallFollower::OnRight:
        std::cout << "Kick Motion [R]: " << ball_position << std::endl;
        playMotion(RightKick);
        printf("right kick \n");
        break;

      case robotis_op::BallFollower::OnLeft:
        std::cout << "Kick Motion [L]: " << ball_position << std::endl;
        playMotion(LeftKick);
        printf("left kick \n");
       break;

      default:
        break;
    }
      on_following_ball_ = false;
      restart_soccer_ = true;
     tracking_status_ = BallTracker::NotFound;
      ball_follower_.clearBallPosition();

      usleep(2000 * 1000);

      if (handleFallen(stand_state_) == true)
        return;

    else
    {

    }
      
  }  

  // ceremony
  //playMotion(Ceremony);
}

void SoccerDemo::handleKick()
{
  usleep(1500 * 1000);

  // change to motion module
  setModuleToDemo("action_module");

  if (handleFallen(stand_state_) == true || enable_ == false)
    return;

  // kick motion
  /*
   control kick angle -10 ~ + 10 

  =====================

  if(ball_tracker_.getPanOfBall() > -10 && ball_tracker_.getPanOfBall() < 10)
  {
    // 1. rotate cw(0~10) or ccw(-10~0) 
    // 2. move only y axis 
  }
  <pjh, ljh>
  */ 

  ball_follower_.decideBallPositin(ball_tracker_.getPanOfBall(), ball_tracker_.getTiltOfBall()); 

  int ball_position = ball_follower_.getBallPosition();
  if(ball_position == BallFollower::NotFound || ball_position == BallFollower::OutOfRange)
  {
    on_following_ball_ = false;
    restart_soccer_ = true;
    tracking_status_ = BallTracker::NotFound;
    ball_follower_.clearBallPosition();
    return;
  }

  switch (ball_position)
  {
  case robotis_op::BallFollower::OnRight:
    std::cout << "Kick Motion [R]: " << ball_position << std::endl;
    sendDebugTopic("Kick the ball using Right foot");
    playMotion(RightKick);
    break;

  case robotis_op::BallFollower::OnLeft:
    std::cout << "Kick Motion [L]: " << ball_position << std::endl;
    sendDebugTopic("Kick the ball using Left foot");
    playMotion(LeftKick);
    break;

  default:
    break;
  }

  on_following_ball_ = false;
  restart_soccer_ = true;
  tracking_status_ = BallTracker::NotFound;
  ball_follower_.clearBallPosition();

  usleep(2000 * 1000);

  if (handleFallen(stand_state_) == true)
    return;

  // ceremony
  //playMotion(Ceremony);
}

bool SoccerDemo::handleFallen(int fallen_status)
{
  if (fallen_status == Stand)
  {
    return false;
  }

  // change to motion module
  setModuleToDemo("action_module");

  // getup motion
  switch (fallen_status)
  {
  case Fallen_Forward:
    std::cout << "Getup Motion [F]: " << std::endl;
    playMotion(is_grass_ ? GetUpFront + ForGrass : GetUpFront);
    break;

  case Fallen_Behind:
    std::cout << "Getup Motion [B]: " << std::endl;
    playMotion(is_grass_ ? GetUpBack + ForGrass : GetUpBack);
    break;

  default:
    break;
  }

  while(isActionRunning() == true)
    usleep(100 * 1000);

  usleep(650 * 1000);

  if (on_following_ball_ == true)
    restart_soccer_ = true;

  // reset state
  on_following_ball_ = false;

  return true;
}

void SoccerDemo::playMotion(int motion_index)
{
  std_msgs::Int32 motion_msg;
  motion_msg.data = motion_index;

  motion_index_pub_.publish(motion_msg);
}

void SoccerDemo::setRGBLED(int blue, int green, int red)
{
  int led_full_unit = 0x1F;
  int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
  robotis_controller_msgs::SyncWriteItem syncwrite_msg;
  syncwrite_msg.item_name = "LED_RGB";
  syncwrite_msg.joint_name.push_back("open-cr");
  syncwrite_msg.value.push_back(led_value);

  rgb_led_pub_.publish(syncwrite_msg);
}

// check running of action
bool SoccerDemo::isActionRunning()
{
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client_.call(is_running_srv) == false)
  {
    ROS_ERROR("Failed to get action status");
    return true;
  }
  else
  {
    if (is_running_srv.response.is_running == true)
    {
      return true;
    }
  }

  return false;
}

void SoccerDemo::sendDebugTopic(const std::string &msgs)
{
  std_msgs::String debug_msg;
  debug_msg.data = msgs;

  test_pub_.publish(debug_msg);
}

}