#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from moveit_commander.robot_trajectory import RobotTrajectory

# Initialize the MoveIt! commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('baxter_assembly_task', anonymous=True)

# Instantiate RobotCommander (interface to the robot)
robot = moveit_commander.RobotCommander()

# Instantiate PlanningSceneInterface (interface to the world)
scene = moveit_commander.PlanningSceneInterface()

# Group for the left arm (adjust if needed)
left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

# Set the reference frame
left_arm_group.set_pose_reference_frame("base")
right_arm_group.set_pose_reference_frame("base")

# Set tolerance
left_arm_group.set_goal_position_tolerance(0.01)
left_arm_group.set_goal_orientation_tolerance(0.01)
right_arm_group.set_goal_position_tolerance(0.01)
right_arm_group.set_goal_orientation_tolerance(0.01)

# Set velocities and accelerations
left_arm_group.set_max_velocity_scaling_factor(0.1)
left_arm_group.set_max_acceleration_scaling_factor(0.1)
right_arm_group.set_max_velocity_scaling_factor(0.1)
right_arm_group.set_max_acceleration_scaling_factor(0.1)

# Define a function to move to a specific pose
def move_to_pose(arm_group, pose):
    arm_group.set_pose_target(pose)
    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

def pick_object():
    # Define the left arm pose for picking
    pick_pose_left = geometry_msgs.msg.Pose()
    pick_pose_left.position.x = 0.6
    pick_pose_left.position.y = 0.3
    pick_pose_left.position.z = 0.2
    pick_pose_left.orientation.x = 0.0
    pick_pose_left.orientation.y = 1.0
    pick_pose_left.orientation.z = 0.0
    pick_pose_left.orientation.w = 0.0
    
    # Move left arm to pick position
    move_to_pose(left_arm_group, pick_pose_left)
    
    # Simulate picking with gripper (replace with actual gripper action)
    rospy.sleep(1.0)  # Simulating time for picking
    rospy.loginfo("Object picked with left arm.")

def place_object():
    # Define the right arm pose for placing
    place_pose_right = geometry_msgs.msg.Pose()
    place_pose_right.position.x = 0.7
    place_pose_right.position.y = -0.4
    place_pose_right.position.z = 0.3
    place_pose_right.orientation.x = 0.0
    place_pose_right.orientation.y = 1.0
    place_pose_right.orientation.z = 0.0
    place_pose_right.orientation.w = 0.0
    
    # Move right arm to place position
    move_to_pose(right_arm_group, place_pose_right)
    
    # Simulate placing with gripper (replace with actual gripper action)
    rospy.sleep(1.0)  # Simulating time for placing
    rospy.loginfo("Object placed with right arm.")

def perform_assembly_task():
    rospy.loginfo("Starting assembly task...")
    
    # Move to initial position
    left_arm_group.set_named_target("left_neutral")
    left_arm_group.go(wait=True)
    right_arm_group.set_named_target("right_neutral")
    right_arm_group.go(wait=True)

    # Perform picking with the left arm
    pick_object()

    # Simulate assembly operations (could be inserting a part)
    rospy.sleep(2.0)

    # Place the object with the right arm
    place_object()

    rospy.loginfo("Assembly task completed!")

if __name__ == '__main__':
    try:
        perform_assembly_task()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
