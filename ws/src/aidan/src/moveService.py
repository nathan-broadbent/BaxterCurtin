#!/usr/bin/env python3

import argparse
import rospy
import numpy as np

from std_msgs.msg import (
    UInt16,
    Header,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

from aidan.srv import(
    armPos
)

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

move = None #Holder for object

class BaseMovement(object):
    
    def __init__(self):
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_arm = baxter_interface.limb.Limb("left")
        # control parameters
        self._rate = 500  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
    
        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in np.arange(100):
            if rospy.is_shutdown():
                return False
            self._right_arm.exit_control_mode()
            self._left_arm.exit_control_mode()
            self._pub_rate.publish(100)
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._right_arm.move_to_neutral()
        self._left_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting...")
        #return to normal
        self._reset_control_modes()
        # self.set_neutral() #Removed As Not Wanted
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def _ik_request(self,limb,pose):
        """
        Return: IK joint angle solutions for a given pose, as list
        Argument: Limb, as a string. Pose, as defined by RR SDK/Wiki
        """
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            rospy.wait_for_service(ns,5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if resp_seeds[0] != resp.RESULT_INVALID:
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints
    
    def moveToCoords(self, armName, pose, rel_cart=False):
        # print(armName)
        # print(pose)
        if not rel_cart:
            joint_angles = self._ik_request(armName,pose)
            print("In Not rel_cart")
        else:
            #if Relative True: Pose becomes offset, orientation is still specified as normal
            #Dictionary containing  pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
            cur_pos = self._right_arm.endpoint_pose()
            # print(f"x value contained: {pose.position.x}")
            pose.position.x = float(cur_pos["position"][0]) + pose.position.x
            pose.position.y = float(cur_pos["position"][1]) + pose.position.y
            pose.position.z = float(cur_pos["position"][2]) + pose.position.z
            # print(f"final position calculated: {pose}")
            joint_angles = self._ik_request(armName,pose)
            
        print(f"Moving To {pose}")
        if joint_angles:
            if(armName == "left"):
                self._left_arm.move_to_joint_positions(joint_angles)
            elif(armName == "right"):
                self._right_arm.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided.")

 
def handle_move_to_pos(req):
    # print(f"MovingTo {req}")
    move.moveToCoords(req.arm, req.pose, req.rel_cart)
    return True

            
def moveToPosServer():
    global move
    rospy.init_node("move_to_pos_server")
    s = rospy.Service('move_to_pos', armPos, handle_move_to_pos)
    move = BaseMovement()
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)
    if(exiting != True):
        # move.moveToCoords('right', [0.5, -0.8, 0.13, -0.478, -0.478, -0.521, -0.521])
        print("Ready To Accept Movement Commands")
        rospy.spin()


if __name__ == '__main__':
    moveToPosServer()
