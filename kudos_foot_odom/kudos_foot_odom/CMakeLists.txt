cmake_minimum_required(VERSION 3.0.2)
project(foot_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  op3_walking_module_msgs
  # sensor_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES foot_odom
  CATKIN_DEPENDS roscpp std_msgs op3_walking_module_msgs
  DEPENDS system_lib
)

include_directories(
  include/${PROJECT_NAME} 
  ${catkin_INCLUDE_DIRS}
)

add_executable(foot_odom_node src/foot_odom_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


