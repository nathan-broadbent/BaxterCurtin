#!/usr/bin/env python

# adding 3.9+ style list[type] annotations
from __future__ import annotations

# General libraries
import sys
import rospy
import actionlib

# Import MoveIt libs
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

# Import Robitiq libs
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

# Import Barret libs
from std_srvs.srv import Empty

#importing Aiden's code
from aidan.srv import (
    armPos,
    probe,
    searchHole
)

# constants
moveit_tolerance = 0.01
left_hand_links = ["left_inner_knuckle", "left_inner_finger", "left_inner_finger_pad", 
                   "right_inner_knuckle", "right_inner_finger", "right_inner_finger_pad"]
right_hand_links = ["bh_base_link", 
                    "bh_finger_11_link", "bh_finger_12_link", "bh_finger_13_link", 
                    "bh_finger_21_link", "bh_finger_22_link", "bh_finger_23_link", 
                    "bh_finger_31_link", "bh_finger_32_link", "bh_finger_33_link"]

def add_box(name: str, pose: list[float], size: list[float]):
    """
    Adds a box to the planning scene
    :param name: Name of the object
    :param pose: Pose of the object as a list [x, y, z]
    :param size: Size of the box as a list [x, y, z]
    :return: Name of the box
    """
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.position.x = pose[0]
    box_pose.pose.position.y = pose[1]
    box_pose.pose.position.z = pose[2]
    box_pose.pose.orientation.w = 1.0
    scene.add_box(name, box_pose, size)
    return name

def add_cyl(name: str, pose: list[float], size: list[float]):
    """
    Adds a cylinder to the planning scene
    :param name: Name of the object
    :param pose: Pose of the object as a list [x, y, z]
    :param size: Size of the cylinder as a list [radius, height]
    :return: Name of the cylinder
    """
    cyl_pose = PoseStamped()
    cyl_pose.header.frame_id = "world"
    cyl_pose.pose.position.x = pose[0]
    cyl_pose.pose.position.y = pose[1]
    cyl_pose.pose.position.z = pose[2]
    cyl_pose.pose.orientation.w = 1.0
    scene.add_cylinder(name, cyl_pose, size[0], size[1])
    return name

def attach_object(object_name: str, link_name: str, dettach: bool = False):
    """
    Attaches or dettaches an object to the robot's end effector.
    :param object_name: Name of the object to attach
    :param link_name: Name of the link to attach the object to
    :param detach: If True, detaches the object; otherwise and by default attaches it
    """
    attached_object = AttachedCollisionObject()
    attached_object.link_name = link_name
    attached_object.object.id = object_name
    attached_object.touch_links = left_hand_links if link_name == "left_gripper" else right_hand_links
    attached_object.object.operation = CollisionObject.REMOVE if dettach else CollisionObject.ADD
    scene.attach_object(attached_object, touch_links=attached_object.touch_links)
    

def move_to_joints(move_group: MoveGroupCommander, pose: list[float]):
    """
    Moves the specified MoveGroup to the given joint pose.
    :param move_group: MoveGroupCommander for the arm
    :param pose: List of joint values to move to. For the arms, a 7-joint list is expected
    """
    move_group.set_joint_value_target(pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

def move_to_pose(move_group: MoveGroupCommander, pose: Pose):
    """
    Moves the specified MoveGroup to the given cartesian pose.
    :param move_group: MoveGroupCommander for the arm
    :param pose: Pose to move to
    """
    move_group.set_pose_target(pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()



from aidan.srv import (
    armPos,
    probe,
    searchHole
)

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)


def moveToPosClient(arm, pose, rel_cart):
    rospy.wait_for_service('/move_to_pos')
    try:
        move_to_pos = rospy.ServiceProxy('/move_to_pos', armPos)
        resp = move_to_pos(arm, pose, rel_cart)
        return resp
    except rospy.ServiceException as e:
        print("service Call Failed: %s"%e)
        return False

def probeSurfaceClient():
    rospy.wait_for_service('/probe_surface')
    try:
        probe_surface = rospy.ServiceProxy('/probe_surface', probe)
        resp = probe_surface()
        return resp
    except rospy.ServiceException as e:
        print("service Call Failed: %s"%e)
        return False
    
def searchHoleClient(xPos, yPos, zHeight, searchWidth, searchLength):
    rospy.wait_for_service('/search_hole')
    try:
        search_hole = rospy.ServiceProxy('/search_hole', searchHole)
        resp = search_hole(float(xPos), float(yPos), float(zHeight), float(searchWidth), float(searchLength))
        return resp
    except rospy.ServiceException as e:
        print("service Call Failed: %s"%e)
        return False

def createPos(pos):
    pose = Pose()
    pose.position.x = float(pos[0])
    pose.position.y = float(pos[1])
    pose.position.z = float(pos[2])
    pose.orientation.x = float(pos[3])
    pose.orientation.y = float(pos[4])
    pose.orientation.z = float(pos[5])
    pose.orientation.w = float(pos[6])
    # print("Finished Conversions")
    return poseopen_grasp

def moveTo(pos):
    # replace with MoveGroupCommander.set_pose_target() - see move_to_pose
    pose = createPos(pos)
    arm = 'right'
    moveToPosClient(arm, pose, False)

def moveToRelative(pos):
    # replace with MoveGroupCommander.shift_pose_target()
    pose = createPos(pos)
    arm = 'right'
    moveToPosClient(arm, pose, True)

def waitForInput(text):
    t = input(f"{text} (q to exit)")

    if len(t) > 0 and t[0].lower() == 'q':
        print("Exiting...")
        raise KeyboardInterrupt("User requested exit")

if __name__ == "__main__":
    left_pose_seq = {
        "untucked":     [-0.091, -1.004, -1.171, +1.943, +0.668, +1.027, -0.497],
        "pre-pickup":   [-1.111, -0.725, +0.930, +1.007, -0.670, +1.529, +0.326],
        "pickup":       [-1.126, -0.674, +0.908, +1.007, -0.670, +1.529, +0.326],
        "retreat":      [-0.297, -0.717, -1.074, +0.811, +0.777, +1.790, -0.267],
        "pre-assemble": [-0.022, +0.311, -1.404, +0.843, +0.059, +1.401, +0.319],
        "assemble":     [-0.022, +0.311, -1.404, +0.843, +0.059, +1.401, +0.319],
        "release":      [-0.022, +0.311, -1.404, +0.843, +0.059, +1.401, +0.319],
    }
    right_pose_seq = {
        "untucked":     [+0.081, -1.002, +1.188, +1.943, -0.669, +1.030, +0.499],
        "pre-pickup":   [-0.732, -0.119, +1.158, +1.506, +0.477, +0.927, -0.470],
        "pickup":       [-0.543, -0.038, +1.154, +1.666, -2.319, -0.620, +2.268],
        "retreat":      [-0.629, -0.351, +1.299, +1.807, -2.256, -0.565, +2.540],
        "pre-assemble": [-0.199, +0.225, +1.866, +1.479, -0.427, +0.888, +0.363],
        "assemble":     [-0.218, +0.429, +1.746, +1.470, -0.489, +0.896, +0.169],
        "release":      [-0.438, +0.025, +1.555, +1.709, +0.043, +0.535, -0.004],
    }

    if len(sys.argv) == 1 or sys.argv[1] not in ["--pick", "--place", "--assemble", "--manual"]:
            # No input, print help message
            print("Usage: python pickup.py [--pick | --place | --assemble | --manual]")

    else:
        print("Initialising...")
        # Initialize the MoveIt! commander and rospy node
        moveit_commander.roscpp_initialize(args="")
        rospy.init_node('baxter_assembly_task', anonymous=True)

        # Instantiate RobotCommander (interface to the robot)
        robot = RobotCommander()

        # Instantiate PlanningSceneInterface (interface to the world)
        scene = PlanningSceneInterface()

        # Group for the left arm (adjust if needed)
        left_arm_group = MoveGroupCommander("left_arm")
        right_arm_group = MoveGroupCommander("right_arm")

        # Set the reference frame
        left_arm_group.set_pose_reference_frame("base")
        right_arm_group.set_pose_reference_frame("base")

        left_arm_group.set_planner_id("RRTConnectkConfigDefault")
        right_arm_group.set_planner_id("RRTConnectkConfigDefault")

        left_arm_group.set_goal_tolerance(moveit_tolerance)
        right_arm_group.set_goal_tolerance(moveit_tolerance)

        print("MoveIt initialised")

        # Robotiq init
        robotiq_client = actionlib.SimpleActionClient("command_robotiq_action", CommandRobotiqGripperAction)   
        robotiq_client.wait_for_server()  
        print("Connected to Robotiq gripper")

        # Robotiq defined positions
        robotiq_closed = CommandRobotiqGripperGoal()
        robotiq_closed.emergency_release = False # Should this be True?
        robotiq_closed.stop = False
        robotiq_closed.position = 0.00
        robotiq_closed.speed = 0.1
        robotiq_closed.force = 5.0

        robotiq_open = CommandRobotiqGripperGoal()
        robotiq_open.emergency_release = False # Should this be True?
        robotiq_open.stop = False
        robotiq_open.position = 0.085
        robotiq_open.speed = 0.1
        robotiq_open.force = 5.0
        # Barret init
        try:
            rospy.wait_for_service("/bhand/initialize")
            barrett_init = rospy.ServiceProxy("/bhand/initialize", Empty)
            rospy.wait_for_service("/bhand/close_grasp")
            barrett_close = rospy.ServiceProxy("/bhand/close_grasp", Empty)
            rospy.wait_for_service("/bhand/open_grasp")
            barrett_open = rospy.ServiceProxy("/bhand/open_grasp", Empty)
            print("Connected to Barrett hand")
            barrett_init()
        except rospy.ROSException as e:
            print(f"Failed to init Barrett hand: {e}")
            sys.exit(1)

        
        try:
            print("complete")
            print("Setting up the scene...")
            scene.clear()
            

            #Add the table to the scene (1500mm x 700mm x 20mm)
            table = add_box("table", [0.85, -0.33, 0.01], [0.70, 1.5, 0.02])
            
            # Approximating the jig to ensure vertical movement and reduce jams. Removed as not useful
            add_box("clamps_and_left_jig", [0.775, 0.42, 0.07], [0.30, 0.18, 0.14])
            #add_box("middle_jig", [0.775, -0.02, 0.07], [0.18, 0.60, 0.10])
            #add_box("right_jig", [0.775, -0.41, 0.07], [0.18, 0.10, 0.10])

            # Add the pipe to the scene (33mm dia, 300mm long). 17cm height to also be above table
            pipe = add_cyl("pipe", [0.775, -0.34, 0.17], [0.3, 0.0165])

            # add the board for the assembly task (200mm x 89mm x 19mm). 12cm height to also be above table
            board_pickup = add_box("board", [0.775, 0.31, 0.12], [0.089, 0.019, 0.2])

            # handling restart if code was run and exited while items were attached
            #attach_object("board", "left_gripper", dettach=True)  
            #attach_object("pipe", "bh_base_link", dettach=True) 


            rospy.sleep(2)  # Wait for the scene to update     
            print("complete")
        except rospy.ROSException as e:
            print(f"Failed to setup scene: {e}")
            sys.exit(1)

        try:
            if (sys.argv[1] == "--pick"):
                #initial movement sequence
                waitForInput("Press enter to open grippers")
                robotiq_client.send_goal(robotiq_open)
                robotiq_client.wait_for_result()
                barrett_open()

                for step in ["untucked", "pre-pickup", "pickup"]:
                    waitForInput(f"Press enter to begin moving to {step} pose")
                    move_to_joints(left_arm_group, left_pose_seq[step])
                    move_to_joints(right_arm_group, right_pose_seq[step])
                    rospy.sleep(1)
            
                waitForInput("Press enter to grasp the parts")
                # Attach objects to arms
                robotiq_client.send_goal(robotiq_closed)
                robotiq_client.wait_for_result()
                barrett_close()

                attach_object("board", "left_gripper")
                attach_object("pipe", "bh_base_link")
                rospy.sleep(1)

                barrett_close() # closing a second time as the pipe is usually loose

                # Retreat to remove parts from jig, then move to assembly position
                for step in ["retreat", "pre-assemble", "assemble"]:
                    waitForInput(f"Press enter to begin moving to {step} pose")
                    move_to_joints(left_arm_group, left_pose_seq[step])
                    move_to_joints(right_arm_group, right_pose_seq[step])
                    rospy.sleep(1)

            elif (sys.argv[1] == "--place"):
                board_place = add_box("board", [0.775, 0.15, 0.03], [0.089, 0.2, 0.019])
                board_pose = scene.get_object_poses(["board"])
                # hole pos relative to board centre
                hole_x_pos = board_pose["board"].position.x - 0.005 - 0.10 #hole is 5cm from centreline, and pipe is 10cm from wrist
                hole_y_pos = board_pose["board"].position.y - 0.045
                angle_offset = 0.05 # offset of pipe tip from centre of wrist, in -y
                # could be dynamically calculated from pipe pose (orientation + known rotation + length of pipe)
                searchWidth = 0.05 #5cm of 9cm board
                searchLength = 0.05 #5cm of 20cm board, but hole is 35mm wide

                #25cm above to account for max potential height of the pipe + 5cm to ensure it is above the surface
                #10cm offset to account for distance from the pipe centre to the wrist location (the enpoint of the arm)
                # orientation is hand out, wrist toward the torso, aligned with the x-axis of the world space
                # y offset is to account for the slightly angled decent the probing code makes (it drifts to baxter's right as it probes down)
                probe_pos = [board_pose["board"].position.x - 0.10, board_pose["board"].position.y + 0.05, board_pose["board"].position.z + 0.25, -0.478, -0.478, -0.521, -0.521]
                
                # it would be better to calculate the delta between the arm pose and pipe pose after pickup to determine actual position
                # useful functions to achieve this:
                #   right_arm_group.get_current_pose()
                #   scene.get_object_poses(["pipe"])
                # or upgrade the ffb coe to use moveit, then reassign the end effector on pickup to the pipe rather than the wrist
                # then reset after releasing the part
                #   right_arm_group.set_end_effector_link("pipe")
                
                waitForInput("Press enter to move right arm to grasp position")
                move_to_joints(right_arm_group, right_pose_seq["retreat"])

                waitForInput("Press enter to close barrett hand")
                barrett_close()
                attach_object("pipe", "bh_base_link")
                #"""
                waitForInput("Press enter to start assembly")
                print("--Moving to probing position")
                # Calls to Aidan's code
                moveTo(probe_pos)

                waitForInput("Press enter to probe surface height")
                resp = probeSurfaceClient()
                # from testing this should be ~0.21
                zHeight = resp.zHeight - 0.005
                print(f"--zHeight: {zHeight}")
                
                waitForInput("Press enter to start search sequence")
                moveTo([hole_x_pos, hole_y_pos+angle_offset, zHeight + 0.1, -0.478, -0.478, -0.521, -0.521])
                searchHoleClient(hole_x_pos, hole_y_pos+angle_offset,zHeight,searchWidth, searchLength)
                print("--Correcting Orientation for Insertion") 
                moveToRelative([0.005,-angle_offset,0,-0.478, -0.478, -0.521, -0.521])
                
                #Final Insert Operation
                waitForInput("Press enter to insert pipe")
                moveToRelative([0,0,-0.01,-0.478, -0.478, -0.521, -0.521]) #
                # End Aidan's code

                waitForInput("Press enter to release pipe")
                barrett_open()
                attach_object("pipe", "bh_base_link", dettach=True)
                moveToRelative([0,0,0.1,-0.478, -0.478, -0.521, -0.521])# moving away from placed pipe
                ##"""
                waitForInput("Press enter to move arm out of the way")
                move_to_joints(right_arm_group, right_pose_seq["release"])
                move_to_joints(left_arm_group, left_pose_seq["release"])

            elif (sys.argv[1] == "--assemble"):
                raise NotImplementedError("Assembly task not implemented yet. Please use --pick or --place.")
                # Assembly code is pick and place in sequence.
            elif (sys.argv[1] == "--manual"):
                while True:
                    cmd = waitForInput("Enter arm to move (l/r) or 'q' to quit: ")
                    if cmd.lower() == 'q':
                        break
                    elif cmd.lower() == 'l':
                        joints = waitForInput("Enter 7 joint values separated by spaces or 'c' or 'o' to close/open the gripper: ").split(", ")
                        if len(joints) == 1 and joints[0].lower() in ['c', 'o']:
                            if joints[0].lower() == 'c':
                                print("Closing gripper")
                                robotiq_client.send_goal(robotiq_closed)
                                robotiq_client.wait_for_result()
                                attach_object("board", "left_gripper")
                            else:
                                print("Opening gripper")
                                robotiq_client.send_goal(robotiq_open)
                                robotiq_client.wait_for_result()
                                attach_object("board", "left_gripper", dettach=True)
                        elif len(joints) != 7:
                            print("Please enter exactly 7 joint values.")
                            continue
                        move_to_joints(left_arm_group, [float(joint) for joint in joints])
                    elif cmd.lower() == 'r':
                        joints = waitForInput("Enter 7 joint values separated by spaces or 'c' or 'o' to close/open the hand: ").split(", ")
                        if len(joints) == 1 and joints[0].lower() in ['c', 'o']:
                            if joints[0].lower() == 'c':
                                print("Closing hand")
                                barrett_close()
                                attach_object("pipe", "bh_base_link")
                            else:
                                print("Opening hand")
                                barrett_open()
                                attach_object("pipe", "bh_base_link", dettach=True)
                        elif len(joints) != 7:
                            print("Please enter exactly 7 joint values.")
                            continue
                        if len(joints) != 7:
                            print("Please enter exactly 7 joint values.")
                            continue
                        move_to_joints(right_arm_group, [float(joint) for joint in joints])
                    else:
                        print("Invalid command. Please enter 'l', 'r', or 'q'.")
            else:
                print("Invalid argument. Use --pick, --assemble, --test, or --probe.")
                sys.exit(1)
            
            waitForInput("Task complete. Press enter to exit")
            #"""

        
        except rospy.ROSInterruptException as e:
            print(f"Exception occured: {e.with_traceback}")
            left_arm_group.stop()
            right_arm_group.stop()

        except KeyboardInterrupt:
            print("Shutdown by user. Exiting...")
            left_arm_group.stop()
            right_arm_group.stop()
            
            if input("Do you want to release the grippers? (y/n): ").lower() == 'y':
                robotiq_client.send_goal(robotiq_open)
                barrett_open()
            
            moveit_commander.roscpp_shutdown()
            sys.exit(0)
        
