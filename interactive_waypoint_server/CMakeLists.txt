cmake_minimum_required(VERSION 2.8.3)
project(interactive_waypoint_server)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  rospy
  interactive_waypoint_server_msgs
)

catkin_package(
   CATKIN_DEPENDS 
	interactive_markers
	rospy
	interactive_waypoint_server_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

