cmake_minimum_required(VERSION 3.0.2)
project(anymal_custom_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()
