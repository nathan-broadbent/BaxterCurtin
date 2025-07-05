#!/usr/bin/env python3

import subprocess

import argparse
import struct
import sys
import numpy as np
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from std_msgs.msg import (
    UInt16,
    Header,
)

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

class ArmVelocity(object):
    def __init__(self):
            self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                            UInt16, queue_size=10)
            self._right_arm = baxter_interface.limb.Limb("right")
            self._left_arm = baxter_interface.limb.Limb("left")
            # control parameters
            self._rate = 100.0  # Hz

            print("Getting robot state... ")
            self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
            self._init_state = self._rs.state().enabled
            print("Enabling robot... ")
            self._rs.enable()
        
            # set joint state publishing to 300Hz
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
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True
    
    def moveVel(self, limb, in_pose):
        ik_resp = self.ikSolve(limb, in_pose)
        print(ik_resp.values())
        print(ik_resp.keys())
        cmd = f"rostopic pub /robot/limb/right/joint_command baxter_core_msgs/JointCommand \"{{mode: 1, command: {list(ik_resp.values())}, names: {list(ik_resp.keys())}}}\" -r 10"
        print(cmd)
        # print(f"\n\nrostopic pub /robot/limb/right/joint_command baxter_core_msgs/JointCommand \"{{mode: 1, command: {list(ik_resp.values())}, names: {list(ik_resp.keys())}}}\" -r 10")
        subprocess.run(cmd, shell=True)
        # ik_resp.
        # while(self.clean_shutdown != True):
            # jald = self._left_arm.joint_angles() #Gets joint angles of left arm
            # angles = [jald["left_s0"], jald["left_s1"], jald["left_e0"], 
            #             jald["left_e1"], jald["left_w0"], jald["left_w1"],
            #             jald["left_w2"]]
            
            # names = self._left_arm.joint_names()
            # pos_dict = dict()
            # for idx, name in enumerate(names):
            #     pos_dict[name] = joint_vel[idx]
            
            # print(vel_dict)
            #Set Joint Velocities
            # self._right_arm.set_joint_positions(ik_resp)
            # print(f"Joint Vel: {joint_vel}")
            # print(f"POS: {cur_pos}")

    def ikSolve(self,limb, in_pose):
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        
        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=in_pose,
            ),
            'right': PoseStamped(
                header=hdr,
                pose=in_pose
            ),
        }
        
        ikreq.pose_stamp.append(poses[limb])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # print("\nIK Joint Solution:\n", limb_joints)
            # print("------------------")
            # print("Response Message:\n", resp)
            # print(limb_joints.keys)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

        return limb_joints

def ik_test(limb, poses):
    rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.7,
                    y=0.5,
                    z=0.2,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.7,
                    y=0,
                    z=0.2,
                ),
                orientation=Quaternion(
                    x=1,
                    y=0,
                    z=0,
                    w=0,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print("\nIK Joint Solution:\n", limb_joints)
        print("------------------")
        print("Response Message:\n", resp)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0


def main():
    print("Initializing node... ")
    rospy.init_node("move_to_controller")
    limb = "right"
    pose=Pose(
        position=Point( #0.92, -0.1, 0.1 (def = 0.8,-0.13,0.25)
            # x=0.8,
            # y=-0.13,
            # z=0.25,
            # 
            x=0.786,
            y=-0.128,
            z=0.128,

            # 0.7851096954931418
            # y: -0.10803037707993425
            # z: 0.12799020333919817

        ),
        orientation=Quaternion(
            # x=-0.1473,
            # y=0.9887,
            # z=-0.0112,
            # w=0.0253,
            x=-0.478,
            y=-0.478,
            z=-0.521,
            w=-0.521,
        ),
    )  
    v = ArmVelocity()
    # print(v.ikSolve(limb, pose))
    v.moveVel(limb,pose)
   
   

    # x=-0.478,
    # y=-0.478,
    # z=-0.521,
    # w=-0.521,

if __name__ == '__main__':
    sys.exit(main())
