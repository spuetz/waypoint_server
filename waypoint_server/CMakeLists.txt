cmake_minimum_required(VERSION 2.8.3)
project(waypoint_server)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  interactive_markers
  rospy
  uuid_msgs
  waypoint_msgs
  unique_id
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS 
  diagnostic_msgs
  geometry_msgs
  interactive_markers
  rospy
  uuid_msgs
  waypoint_msgs
  unique_id
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/waypoint_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

