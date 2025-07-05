# barrett-ros-pkg
> ROS package for the Barrett WAM and related products.

- [Overview](#overview)
- [ Pre-Requisites](#pre-requisites)
- [Compiling the package](#compiling-the-package)
- [Running `wam_node`](#running-wam_node)
- [Running `wam_demos`](#running-wam_demos)
- [Running `perception_palm`](#running-perception_palm)
	- [Set up cameras](#set-up-cameras)
	- [Calibration](#calibration)
	- [Running the demo](#running-the-demo)
	- [Accessing the sensors](#accessing-the-sensors)
	 - [LED](#led)
	 - [Laser](#laser)
	 - [IR Range finder](#ir-range-finder)
	 - [Camera](#camera)
    - [Networking](#networking)
	- [Troubleshooting](#troubleshooting)
- [Running `barrett_hand_node`](#running-barrett_hand_node)
- [Example of running the services (Tested on ROS Melodic and Indigo)](#example-of-running-the-services-tested-on-ros-melodic-and-indigo)

## Overview
This is Barrett Technology's ROS repository that wraps Libbarrett's functionalities and includes a ROS driver for Barrett's Perception Palm. The Perception Palm includes a LED, a Laser, two cameras and an IR Range finder. The driver for the Perception palm wraps the open source C/C++ library for Microchip's USB-to-SPI protocol coverter. Libbarrett is a real-time controls library written in C++ that runs Barrett Technology's products, including the WAM Arm and the BH8-282 BarrettHand. 

- The `wam_node` stack is designed to be run on a WAM PC-104 or external control PC, and can work with the WAM with any combination of the wrist, BarrettHand and the Force/Torque sensor attached.
- The `barrett_hand_node` stack is designed to run on an external control PC and can work with the BarrettHand standalone **connected via the CAN bus**.
- The `perception_palm` stack is designed to run on an external control PC.
- The `wam_msgs`, `wam_srvs` and `wam_teleop` stacks are designed as the interface to communicate with the functionality exposed by the` wam_node`.

## Pre-Requisites
#### On  Ubuntu 14.04:

1.  An installed version of [Libbarrett 1.3.0](https://git.barrett.com/software/libbarrett/tree/devel-14.04)

2. [ROS Indigo](https://wiki.ros.org/indigo/Installation/Ubuntu)

3. Install libudev and wstool.
```
sudo apt-get update
sudo apt-get install libudev-dev
sudo apt-get install python-wstool
```
4. Install the camera driver:
```
sudo apt-get install ros-indigo-camera-umd
```

#### On  Ubuntu 18.04:
1. An installed version of [Libbarrett 2.0.0](https://git.barrett.com/software/libbarrett/blob/devel/README.md)

2. [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)

3. Install libudev and wstool.
```
sudo apt-get update
sudo apt-get install libudev-dev
sudo apt-get install python-wstool
```

4. Install the camera driver:
```
sudo apt-get install ros-melodic-camera-umd
```

#### On  Ubuntu 20.04:
1. An installed version of [Libbarrett 3.0.0](https://git.barrett.com/software/libbarrett/blob/devel/README.md)

2. [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) 

3. Install libudev and the camera driver.
```
sudo apt update
sudo apt install libudev-dev ros-noetic-usb-cam
```

## Compiling the package
Create a new Catkin Workspace **if not already done**:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Clone the git repository an run the build script:

```sh
cd ~/catkin_ws/src
git clone https://git.barrett.com/software/barrett-ros-pkg.git
cd barrett-ros-pkg
git checkout devel
sudo -s
./build.sh
exit
```

Source the package before running (does not persist after reboot):
```sh
source ~/catkin_ws/devel/setup.bash
```

OR add it to ```bashrc``` (persists after reboot):
```sh
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Running `wam_node`
Launch the ```wam_node.launch``` file, **with the WAM Arm connected**:
```sh
roslaunch wam_node wam_node.launch
```

## Running `wam_demos`

**To Cycle a 7DOF WAM and BarrettHand 10 times from a shell script:**
```
sh cmds-7dof-cycle.sh 10
```

**To Teach:**
```
rosrun wam_demos teach -t <record_type> -n <bag_name>
```
The -t <record_type> field allows you to choose how to record the trajectories
Possible values are:
- ```-t jp```: Record using joint positions
- ```-t jv```: Record using joint positions

**To Play:**
```
rosrun wam_demos play <bag_name>
```

## Running `perception_palm`

### Set up cameras:
1. Install the camera utilities
```
sudo apt install v4l-utils guvcview
```
2. Connect the Perception Palm to the PC using a USB cable

3. Determine the device names for the Perception Palm cameras. 

List the video devices with:
```
v4l2-ctl --list-devices
```
To determine which devices are correct, you can use a program such as `guvcview`:
```
guvcview -d /dev/video0
```
Note which device corresponds to which camera. Some of the /dev/video devices may actually correspond to a camera's metadata (like image format) instead of representing a video stream. For detailed information on /dev/video0 (for example), use:
```
v4l2-ctl --device=/dev/video0 --all
```
If the "Device Caps" section advertises "Video Capture", then this device has a valid video stream.

4. Edit the launch file to confirm camera setup parameters.
```
gedit ~/catkin_ws/src/barrett-ros-pkg/perception_palm/launch/perception_palm.launch
```
Ensure that the launch file targets the correct devices. If you need to change the default devices, edit the lines in the launch file that look like this:
```
<param name="device" type="string" value="/dev/video0" />
```

*Notes*

A single camera can be used at a maximum resolution of 1600 x 1200. Two cameras are limited to a maximum resolution of 320 x 240 due to how much USB bandwidth is (incorrectly) requested/estimated for each video stream. For information on alternative camera resolutions, formats, and frame rates, use this command:
```
v4l2-ctl --list-formats-ext
```

The camera with the red filter is physically rotated 180 degrees with respect to the other camera. You may want to flip the video stream from this camera.

### Calibration

**IR Range finder**

**The IR range finder needs to be calibrated for the first time before using it.** It will work without calibration but it might not be accurate. Follow the instructions below to calibrate it. This step can be skipped if you do not want to use the IR or if the IR range finder is already calibrated.
```
cd ~/catkin_ws/src/barrett-ros-pkg/perception_palm/include/MCP2210-Library
make
sudo ./dist/Debug/GNU-Linux-x86/ir_calibrate
```
Follow the onscreen instructions.

**Cameras**

Please refer to the ROS [Stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)/[Monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) calibration package. This step is completely optional. The cameras would work even if this step is skipped.

### Running the demo

1.  (every time you plug in the Perception Palm or reboot the computer). For one camera
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```
    or two cameras
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128
```

2. Become the root user to access the drivers and run the demo.
**For one camera:**
```
sudo -s
source /home/robot/catkin_ws/devel/setup.bash
roslaunch perception_palm perception_palm.launch
```

**For two cameras:**
```
sudo -s
source ~/catkin_ws/devel/setup.bash
roslaunch perception_palm perception_palm.launch mono_camera:=false
```

3. To quit, press Ctrl-C. Then type `exit` to return to a regular terminal.

*Troubleshooting*

If the camera node fails to start in step 2, make sure the configuration you chose in step 1 matches the configuration in the launch file. See the "Set up cameras" section for details.

### Accessing the sensors

While the demo is running you can access the sensors from a separate terminal.

#### LED

The LED can be turned off and on by calling the service barrett/palm/set_led_on.<br />
```	
rosservice call barrett/palm/set_led_on ['True']
rosservice call barrett/palm/set_led_on ['False']
```

#### Laser

The Laser can be turned off and on by calling the service barrett/palm/set_laser_on.<br />
```	
rosservice call barrett/palm/set_laser_on ['True']
rosservice call barrett/palm/set_laser_on ['False']
```

#### IR Range finder

The IR range finder publishes the range information as a sensor_msgs/Range message, at 1Hz to the topic barrett/palm/ir/range<br />
```
rostopic echo barrett/palm/ir/range
```

#### Camera

By default, the two camera feeds are published at the rate of 30 fps with a resolution of 320x240 px. They are published to the topics, barrett/palm/left/image_raw and barrett/palm/right/image_Raw.<br />
While using the monocular camera mode, the images are published at the rate of 30 fps with a resolution of 1600 x 1200 to the topic barrett/palm/image_raw.<br />

### Networking

This is required if you use run the ROS nodes across multiple computers. The below instructions are to setup the perception palm on the XWAM and to control it from a remote host.

In the host computer, open a new terminal and ssh into the XWAM
```
ssh summit@*xwam's ip address*
```
Enter the XWAM's password and open the hosts file
```
sudo vi /etc/hosts
```
Add the IP address of the remote host machine to the list and name it
```
*remote host's ip address* *remote host's name*
```
Save and exit it. Set the XWAM to be the ROS MASTER
```
export ROS_MASTER_URI=http://*XWAM's ip address*:11311
```
Open a new terminal and edit the host file in the host machine as above
```
sudo vi /etc/hosts
```
In the host machine, name the IP address of the xwam
```
*xwam's ip address* summit
```
Save and exit it. Update the ROS_MASTER as the XWAM in the host machine
```
export ROS_MASTER_URI=http://*XWAM's ip address*:11311
```

P.S: The ROS_MASTER_URI variable would be active as long as the terminal is open. If the terminal is closed and restarted, the ROS_MASTER_URI needs to be updated.
For more information on ROS network configuration refer to [ROS documentation](http://wiki.ros.org/ROS/NetworkSetup).

### Troubleshooting

Trying to restart the perception_palm package multiple times might fail with an error "Error setting SPI Parameters".
This can be solved by unplugging and plugging back the USB to the port.

## Running `barrett_hand_node`

Launch the ```barrett_hand_node.launch``` file, with the **BarrettHand connected via the CAN bus**:
```sh
roslaunch barrett_hand_node barrett_hand_node.launch
```

In a separate terminal:
```sh
rosservice call /bhand/close_grasp
rosservice call /bhand/open_grasp
rosservice call /bhand/grasp_pos 1.57
rosservice call /bhand/finger_pos "[0.5, 1, 1.5]"
rosservice call /bhand/initialize
rosservice call /bhand/spread_pos 1.57
rosservice call /bhand/open_spread

rostopic echo /bhand/finger_tip_states
rostopic echo /bhnad/joint_states
rostopic echo /bhand/tactile_states
```

## Examples of running the WAM services (Tested on ROS Melodic and Indigo)
**Move WAM Joints:**
```
rosservice call /wam/joint_move "joints:
- 0.0
- 0.0
- 0.0
- 0.0"
```

**Move WAM to Tool Pose:**
```
rosservice call /wam/pose_move "pose:
  position:
    x: 0.048
    y: 0.0
    z: 0.618
  orientation:
    x: -0.0190
    y: 0.9022
    z: -0.2516
    w: -0.3498"
```

**Move WAM Home:**
```
rosservice call /wam/go_home
```

**Hold Joint Positions:**
```
rosservice call /wam/hold_joint_pos "hold: true"
```

**Unhold Joint Positions:**
```
rosservice call /wam/hold_joint_pos "hold: false"
```
