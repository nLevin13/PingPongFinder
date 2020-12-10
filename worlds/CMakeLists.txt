cmake_minimum_required(VERSION 2.8.3)
project(worlds)

find_package(catkin REQUIRED COMPONENTS
  rospy
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  message_generation
)

add_service_files(
  FILES
  MapAndEndpts.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    visualization_msgs
    geometry_msgs
    std_msgs
    tf2_ros
  DEPENDS
    message_runtime
)
