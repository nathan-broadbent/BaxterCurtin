# Setup Notes for Baxter

## Comms Setup
Confirm comms are working between the workstation and Baxter. 
Baxter's hostname is `011406P0001.local`, and its IP has been configured to a static IP of `134.7.44.203`
```bash
env | grep ROS_MASTER_URI
```
Should return `ROS_MASTER_URI` as `011406P0001.local`, and when pinging it should indicate `134.7.44.203` when pinging it.  
Confirm that the ROS_IP addresses in `baxter.sh` and `~/.bashrc` match with Baxter's IP address.

## Baxter Remote
To remote into Baxter, use the following
``` bash
ssh ruser@011406P0001.local
rethink
```
Check that the workstation can be pinged at `134.7.44.201`.  
Baxter has a Field Service Menu that can be accessed by holding ALT and pressing F twice during boot. For details of this menu, refer to the included PDF []() found originally [here]()
env | grep ROS_MASTER_URI

## Baxter Control
Once the workspace has been built, run the following commands
```bash
. baxter.sh
. devel/setup.sh
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
rosrun baxter_interface joint_trajectory_action_server.py
```
- `enable_robot.py` is used to enable Baxter's motors. `-h` option to list all options, but `-e`nable and `-d`isable are the primary uses.
- `tuck_arms` to `-u`ntuck or `-t`uck Baxter's arms from the storage/home position. `-h` for help/.

## 2F-85 Gripper Control
Once the workspace has been built, run the following commands. This gripper is connected via USB, check it is plugged in and turned on, and `lsusb` to confirm if it's available. The gripper will cycle closed and open when the node is started.
```bash
. devel/setup.bash
. baxter.sh 
roslaunch robotiq_2f_gripper_control robotiq_action_server.launch
```
Code modified from https://github.com/KevinGalassi/Robotiq-2f-85

## BH8-282 Gripper Control
Once the workspace has been built, run the following commands. This gripper is connected via the PEAK USB-CAN adapter. `lsusb` and `ls /sys/class/tty | grep CAN` to check if its connected. IF drivers need to be reinstalled or for troubleshooting see https://www.peak-system.com/fileadmin/media/linux/index.php. Check power supply/controller box switch is set to CAN (not RS-232) mode.
```bash
. devel/setup.bash
. baxter.sh
roslaunch barrett_hand_node barrett_hand_node.launch
```
To use the node manually, use the following
```bash
. baxter.sh
. devel/setup.bash
rosservice call /bhand/initialize
rosservice call /bhand/close_grasp
rosservice call /bhand/open_grasp
```
Currently, the splay axis is not working. This appears to be a mechanical/electrical issue, there is no resistance to motion on the splay fingers.

It also fails occasionally due to the red power wire on the connector on the hand being loose.

## F-T Sensor
This sensor is connected over the network and has been configured with a static IP at `134.7.44.202`. It can be accessed at that IP to change the configuration.
```bash
. baxter.sh
. devel/setup.bash
rosrun ati_sensor_ros_driver ati_driver.py
```
It has the following services and can be visualised witht he WrenchStamped element in RViz.
```bash
rosservice call /set_ati_tare
rosservice call /clear_ati_tare
```
## Aidan's Code

```bash
roslaunch aidan insertion_services.launch 
```

Provides the following services.
- Move to a position with the optomisation controller (accurate but slow and with no collision avoidance) either to some offset of the current location or to a specific end effector pose
- Move vertically down until a force increase indicates a surface has been reached
- Drag an object against a surface until force increases to indicate it is pressing against the inner wall of a hole.
```bash
rosservice call /move_to_pos {Pose:=[x, y, z, rz, ry, rz, rw], relative:=bool}
rosservice call /probe_surface
rosservice call /search_hole {x, y, z, width, length}
```

## MoveIt! Launchfiles
The MoveIt! launch file is for a blank scene, the script creates the appropriate objects to represent the table and parts.
```bash
. baxter.sh
. devel/setup.bash
roslaunch baxter_moveit_config demo_baxter.launch
```


## RViz/MoveIt!/URDF Tools
### MoveIt! Setup Assistant
Used to adjust collisions primarily not that the URDF is working
```bash
. devel/setup.bash
cd /home/baxter/Alasdair_Thesis/ws_AF/src/moveit_robots/baxter/baxter_moveit_config
roslaunch moveit_setup_assistant setup_assistant.launch
```
### URDF rebuild
```
xacro /home/baxter/Alasdair_Thesis/ws_AF/src/baxter_common/baxter_description/urdf/baxter.urdf.xacro > /home/baxter/Alasdair_Thesis/ws_AF/src/baxter_common/baxter_description/urdf/baxter.urdf

xacro `rospack find baxter_moveit_config`/config/baxter.srdf.xacro left_electric_gripper:=false right_electric_gripper:=false left_tip_name:=left_gripper right_tip_name:=right_gripper > /home/baxter/Alasdair_Thesis/ws_AF/src/moveit_robots/baxter/baxter_moveit_config/config/baxter.srdf
```
