# BaxterCurtin

This repo contains the functions and packages for working with the Baxter Robot owned by Curtin University. It was created by Curtin students working on their thesis projects, and serves as a basis for future students working on the Baxter Robot.

## Structure
- setup: detailed notes on bringing up all nodes and controlling baxter
- ws: ROS Noetic workspace containing the code. Contains the `baxter.sh` script to connect to baxter and the following packages:
   - aidan: Code from Aiadan Southwell's thesis for FFB guided assembly. Call `roslaunch aidan insertion_services.launch` to bring up the required nodes
   - ati_sensor_ros_driver: Gets data from the FT sensor and publishes it as a WrenchStamped message to ROS. Call `rosrun ati_sensor_ros_driver ati_driver`
   - barrett-ros-pkg: Manufacturer-provided ROS driver for the BH8. Call `roslaunch barrett-ros-pkg barrett_hand_node.launch` and see Setup for usage
   - baxter_interface: part of the Baxter drivers. Call `rosrun baxter_interface joint_trajectory_action_server` and see Setup for related setup scripts in other Baxter packages (eg: baxter_tools).
   - baxter_tom: Contains a script to run a complete assembly task, with some issues (see the thesis for discussion). Call `rosrun baxter_tom pickup <arg>`. For usage, call with --pick for the pickup task to be executed, --place for the arms to assemble with manual interaction to initially locate the parts (manually move the baxter hand to grab the pipe in the hole on the jig and assuming that the board is clamped above the jig), --assemble is not implemented (see thesis for explaination) and --manual to send joint poses and operate the grippers to test the code
   - moveit_robots: contains URDF files and other MoveIt configuration to bring up the robot in RViz and MoveIt. See Setup for usage
   - robotiq_2f_gripper_control: Robotiq gripper ROS package. Call `roslaunch robotiq_2f_gripper_control robotiq_action_server.launch` and see Setup for usage.

## Instructions
### Build
Install ROS Noetic per the instructions [here](https://wiki.ros.org/noetic/Installation/Ubuntu)  
Inside the workspace, call ```catkin_make```

## See also
Previous theses at the following github repositories:
- Aidan: https://github.com/Macindaw/Baxter2024
- Alasdair: https://github.com/Alasdair28/Alasdair_Thesis
- Tom: https://github.com/Tom41878/BaxterThesis
