#!/usr/bin/env python3

import rospy
from aidan.srv import (
    armPos
)

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)

def moveToPosClient(arm, pose, rel_cart):
    rospy.wait_for_service('move_to_pos')
    try:
        move_to_pos = rospy.ServiceProxy('move_to_pos', armPos)
        resp = move_to_pos(arm, pose, rel_cart)
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
    return pose

def pickTube():
    #Old Z Height = 0.13
    pose1 = createPos([0.5, -0.7, 0.07, -0.478, -0.478, -0.521, -0.521])
    pose2 = createPos([0.6, -0.7, 0.07, -0.478, -0.478, -0.521, -0.521])
    pose3 = createPos([0.6, -0.7, 0.2, -0.478, -0.478, -0.521, -0.521])
    pose4 = createPos([0.8,-0.13,0.25, -0.478, -0.478, -0.521, -0.521])
    arm = 'right'
    moveToPosClient(arm, pose1, False)
    input("Please Enter Any Character To Confirm Arm Opened")
    moveToPosClient(arm, pose2, False)
    input("Please Enter Any Character To Confirm Arm Closed")
    moveToPosClient(arm, pose3, False)
    #Move TO Insertion Location
    moveToPosClient(arm, pose4, False)
    print("Pick and Place Complete")

def pickTubePos(xpos, ypos, zpos):
    pose1 = createPos([xpos-0.1, ypos, zpos, -0.478, -0.478, -0.521, -0.521])
    pose2 = createPos([xpos, ypos, zpos, -0.478, -0.478, -0.521, -0.521])
    pose3 = createPos([xpos, ypos, zpos+0.15, -0.478, -0.478, -0.521, -0.521])
    pose4 = createPos([0.8,-0.13,0.25, -0.478, -0.478, -0.521, -0.521])
    arm = 'right'
    moveToPosClient(arm, pose1, False)
    input("Please Enter Any Character To Confirm Arm Opened")
    moveToPosClient(arm, pose2, False)
    input("Please Enter Any Character To Confirm Arm Closed")
    moveToPosClient(arm, pose3, False)
    #Move TO Insertion Location
    moveToPosClient(arm, pose4, False)
    print("Pick and Place Complete")

def moveTo(pos):
    pose = createPos(pos)
    arm = 'right'
    moveToPosClient(arm, pose, False)

def moveToRelative(pos):
    pose = createPos(pos)
    arm = 'right'
    moveToPosClient(arm, pose, True)


if __name__ == "__main__":
    pickTube()    

