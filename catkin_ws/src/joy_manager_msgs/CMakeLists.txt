cmake_minimum_required(VERSION 3.16.3)
project(joy_manager_msgs)

if("$ENV{ROS_VERSION}" STREQUAL "1")

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    sensor_msgs
    std_msgs
)

add_message_files(
  FILES
    AnyJoy.msg
)
generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_generation
    message_runtime
    sensor_msgs
    std_msgs
)

else() # ROS version 2

set(PACKAGE_DEPENDENCIES
  sensor_msgs
  std_msgs
  )

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

set(MSG_FILES
  "msg/AnyJoy.msg"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
  )

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()

endif()
