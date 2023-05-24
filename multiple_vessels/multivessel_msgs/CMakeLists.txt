cmake_minimum_required(VERSION 2.8.3)
project(multivessel_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  message_runtime
  
)

add_message_files(
   FILES
   VesselPose.msg
   VesselDetails.msg
   Perception.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  trajectory_msgs
)

catkin_package(
   CATKIN_DEPENDS message_generation message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
