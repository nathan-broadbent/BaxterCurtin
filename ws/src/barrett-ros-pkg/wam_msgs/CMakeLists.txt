cmake_minimum_required(VERSION 2.8.3)
project(wam_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files(DIRECTORY msg FILES
  RTCartPos.msg
  FtTorques.msg
  RTOrtnPos.msg
  RTOrtnVel.msg
  RTPose.msg
  tactilePressure.msg
  tactilePressureArray.msg
  RTCartVel.msg
  RTJointPos.msg
  RTJointVel.msg
  EndpointState.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS 
  message_runtime
  geometry_msgs
  sensor_msgs
  std_msgs
)

