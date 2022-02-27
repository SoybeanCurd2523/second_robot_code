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

#ifndef SOCCER_DEMO_H
#define SOCCER_DEMO_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "op3_action_module_msgs/IsRunning.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/SetJointModule.h"

#include "op3_demo/op_demo.h"
#include "op3_demo/ball_tracker.h"
#include "op3_demo/ball_follower.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_ball_detector/kudos_vision_op3_local_mode.h"
#include "op3_ball_detector/kudos_vision_gcm.h"
#include "op3_ball_detector/kudos_vision_mcl2_local_result.h"

// #include "open_cr_module/open_cr_module.h" //pjh

namespace robotis_op
{

class SoccerDemo : public OPDemo
{
 public:
  enum Stand_Status
  {
    Stand = 0,
    Fallen_Forward = 1,
    Fallen_Behind = 2,
  };

  enum Robot_Status
  {
    Waited = 0,
    TrackingAndFollowing = 1,
    ReadyToKick = 2,
    ReadyToCeremony = 3,
    ReadyToGetup = 4,
  };

  SoccerDemo();
  ~SoccerDemo();

  void setDemoEnable();
  void setDemoDisable();

 protected:
  const double FALL_FORWARD_LIMIT;
  const double FALL_BACK_LIMIT;
  const int SPIN_RATE;
  const bool DEBUG_PRINT;

  void processThread();
  void callbackThread();
  void trackingThread();
  void gcm_mode_init();
  void gcm_mode_set();
  void gcm_mode_play();

  void setBodyModuleToDemo(const std::string &body_module, bool with_head_control = true);
  void setModuleToDemo(const std::string &module_name);
  void callServiceSettingModule(const robotis_controller_msgs::JointCtrlModule &modules);
  void parseJointNameFromYaml(const std::string &path);
  bool getJointNameFromID(const int &id, std::string &joint_name);
  bool getIDFromJointName(const std::string &joint_name, int &id);
  int getJointCount();
  bool isHeadJoint(const int &id);
  void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
  void demoCommandCallback(const std_msgs::String::ConstPtr& msg);
  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void gameControllerMessageCallback(const op3_ball_detector::kudos_vision_gcm::ConstPtr &msg);
  void mcl2_local_result_MessageCallback(const op3_ball_detector::kudos_vision_mcl2_local_result::ConstPtr &msg);
  void setWalkingCommand(const std::string &command);
  void sensor_yaw_callback(const std_msgs::Float64::ConstPtr &msg);

  void startSoccerMode();
  void stopSoccerMode();

  void process();
  void handleKick(int ball_position);
  void handleKick();
  bool handleFallen(int fallen_status);

  void playMotion(int motion_index);
  void setRGBLED(int blue, int green, int red);
  bool isActionRunning();

  void sendDebugTopic(const std::string &msgs);

  BallTracker ball_tracker_;
  BallFollower ball_follower_;
  // OpenCRModule open_cr_module_; //pjh

  ros::Publisher module_control_pub_;
  ros::Publisher motion_index_pub_;
  ros::Publisher rgb_led_pub_;
  ros::Publisher kudos_vision_op3_local_mode_msg_;
  ros::Publisher set_walking_command_pub_;
  ros::Subscriber buttuon_sub_;
  ros::Subscriber demo_command_sub_;
  ros::Subscriber imu_data_sub_;
  ros::Subscriber gcm_sub_;
  ros::Subscriber mcl2_sub_;

  ros::Subscriber sensor_yaw_sub_; //kudos

  ros::Publisher test_pub_;

  ros::ServiceClient is_running_client_;
  ros::ServiceClient set_joint_module_client_;

  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  bool is_grass_;
  int wait_count_;
  int pre_gcm_game_state;
  bool on_following_ball_;
  bool on_tracking_ball_;
  bool restart_soccer_;
  bool start_following_;
  bool stop_following_;
  bool stop_fallen_check_;
  int robot_status_;
  int tracking_status_;
  int stand_state_;
  double present_pitch_;

  //About Gcm
  int gcm_game_state;
  int Is_penalty;
  int Secondary_remaining;
  int count;
  bool enable_set_mode_trigger;
  int bh_init_setting;
  int gcm_vision_go_local_init_setting;
  int gcm_vision_go_local;

  //About mcl2
  double local_result_x;
  double local_result_y;
  double local_result_orien;
  int start_point_x;
  int start_point_y;
  int start_point_orien;

  //about imu_yaw_Data
  double kudos_imu_yaw;

};

}  // namespace robotis_op
#endif // SOCCER_DEMO_H