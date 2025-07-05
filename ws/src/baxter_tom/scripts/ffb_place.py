#!/usr/bin/env python3

import rospy
from aidan.srv import (
    armPos,
    probe,
    searchHole
)

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)

import pickup

def moveToPosClient(arm, pose, rel_cart):
    rospy.wait_for_service('move_to_pos')
    try:
        move_to_pos = rospy.ServiceProxy('move_to_pos', armPos)
        resp = move_to_pos(arm, pose, rel_cart)
        return resp
    except rospy.ServiceException as e:
        print("service Call Failed: %s"%e)
        return False

def probeSurfaceClient():
    rospy.wait_for_service('probe_surface')
    try:
        probe_surface = rospy.ServiceProxy('probe_surface', probe)
        resp = probe_surface()
        return resp
    except rospy.ServiceException as e:
        print("service Call Failed: %s"%e)
        return False
    
def searchHoleClient(xPos, yPos, zHeight, searchWidth, searchLength):
    rospy.wait_for_service('search_hole')
    try:
        search_hole = rospy.ServiceProxy('search_hole', searchHole)
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
    return pose

def moveTo(pos):
    pose = createPos(pos)
    arm = 'right'
    moveToPosClient(arm, pose, False)

def moveToRelative(pos):
    pose = createPos(pos)
    arm = 'right'
    moveToPosClient(arm, pose, True)


