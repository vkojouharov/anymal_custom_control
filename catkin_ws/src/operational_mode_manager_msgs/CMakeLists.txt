cmake_minimum_required(VERSION 3.16.3)
project(operational_mode_manager_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    std_msgs
    actionlib_msgs
)

add_message_files(
  FILES
    OperationalMode.msg
)

add_action_files(
  FILES
    SwitchOperationalMode.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    actionlib_msgs
    operational_mode_manager_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
    actionlib_msgs
)
