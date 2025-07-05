cmake_minimum_required(VERSION 3.0.2)
project(baxter_tom)

find_package(catkin
    REQUIRED
    COMPONENTS
    rospy
    actionlib
    geometry_msgs
    moveit_msgs
    baxter_core_msgs
    baxter_interface
    moveit_commander
    robotiq_2f_gripper_msgs
    robotiq_2f_gripper_control)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
    actionlib
    geometry_msgs
    moveit_msgs
    baxter_core_msgs
    baxter_interface
    moveit_commander
    robotiq_2f_gripper_msgs
    robotiq_2f_gripper_control
)
install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
