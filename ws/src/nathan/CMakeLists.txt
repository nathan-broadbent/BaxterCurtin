cmake_minimum_required(VERSION 3.0.2)
project(nathan)

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
    robotiq_2f_gripper_control
    ati_sensor_ros_driver
    )

add_service_files(
  FILES
  Probe.srv
  SearchHole.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
    ati_sensor_ros_driver
    message_runtime
)
install(
  DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
  USE_SOURCE_PERMISSIONS
)
