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

catkin_install_python(PROGRAMS
  cable_outfitting/run_cable_trajectory.py
  cable_outfitting/list_oakd_cameras.py
  cable_outfitting/debug/run_cable_outfitting.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY cable_outfitting/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cable_outfitting/config
)

install(FILES cable_outfitting/README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cable_outfitting
)
