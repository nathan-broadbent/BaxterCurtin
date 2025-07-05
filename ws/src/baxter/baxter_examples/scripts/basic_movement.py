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

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

def namCheckandReturn(text,bigIter,smalIter,arr,suf, pre):
    if(text[bigIter+smalIter][-2:] == suf and text[bigIter+smalIter][:3] == pre):
        name = text[bigIter+smalIter]
        value = text[bigIter+smalIter+2]
        if(arr is None):
            arr = np.array([name,value])
        else:
            arr = np.append(arr,[name,value])

    return(arr)


def read(file):
    f = open(file, "r")
    words =  f.read().split()

    pos_1 = pos_2 = pos_3 = pos_4 = pos_5 = pos_6 = None

    for i, word in enumerate(words):
        if(word == 'position_1:'):
            for j in range(21):
                pos_1 = namCheckandReturn(words,i,j,pos_1,".x","pos")
                pos_1 = namCheckandReturn(words,i,j,pos_1,".y","pos")
                pos_1 = namCheckandReturn(words,i,j,pos_1,".z","pos")
                pos_1 = namCheckandReturn(words,i,j,pos_1,".x","ori")
                pos_1 = namCheckandReturn(words,i,j,pos_1,".y","ori")
                pos_1 = namCheckandReturn(words,i,j,pos_1,".z","ori")
                pos_1 = namCheckandReturn(words,i,j,pos_1,".w","ori")
        if(word == 'position_6:'):
            for j in range(21):
                pos_6 = namCheckandReturn(words,i,j,pos_6,".x","pos")
                pos_6 = namCheckandReturn(words,i,j,pos_6,".y","pos")
                pos_6 = namCheckandReturn(words,i,j,pos_6,".z","pos")
                pos_6 = namCheckandReturn(words,i,j,pos_6,".x","ori")
                pos_6 = namCheckandReturn(words,i,j,pos_6,".y","ori")
                pos_6 = namCheckandReturn(words,i,j,pos_6,".z","ori")
                pos_6 = namCheckandReturn(words,i,j,pos_6,".w","ori")
        if(word == 'position_2:'):
            for j in range(21):
                pos_2 = namCheckandReturn(words,i,j,pos_2,".x","pos")
                pos_2 = namCheckandReturn(words,i,j,pos_2,".y","pos")
                pos_2 = namCheckandReturn(words,i,j,pos_2,".z","pos")
                pos_2 = namCheckandReturn(words,i,j,pos_2,".x","ori")
                pos_2 = namCheckandReturn(words,i,j,pos_2,".y","ori")
                pos_2 = namCheckandReturn(words,i,j,pos_2,".z","ori")
                pos_2 = namCheckandReturn(words,i,j,pos_2,".w","ori")
        if(word == 'position_5:'):
            for j in range(21):
                pos_5 = namCheckandReturn(words,i,j,pos_5,".x","pos")
                pos_5 = namCheckandReturn(words,i,j,pos_5,".y","pos")
                pos_5 = namCheckandReturn(words,i,j,pos_5,".z","pos")
                pos_5 = namCheckandReturn(words,i,j,pos_5,".x","ori")
                pos_5 = namCheckandReturn(words,i,j,pos_5,".y","ori")
                pos_5 = namCheckandReturn(words,i,j,pos_5,".z","ori")
                pos_5 = namCheckandReturn(words,i,j,pos_5,".w","ori")
        if(word == 'position_3:'):
            for j in range(21):
                pos_3 = namCheckandReturn(words,i,j,pos_3,".x","pos")
                pos_3 = namCheckandReturn(words,i,j,pos_3,".y","pos")
                pos_3 = namCheckandReturn(words,i,j,pos_3,".z","pos")
                pos_3 = namCheckandReturn(words,i,j,pos_3,".x","ori")
                pos_3 = namCheckandReturn(words,i,j,pos_3,".y","ori")
                pos_3 = namCheckandReturn(words,i,j,pos_3,".z","ori")
                pos_3 = namCheckandReturn(words,i,j,pos_3,".w","ori")
        if(word == 'position_4:'):
            for j in range(21):
                pos_4 = namCheckandReturn(words,i,j,pos_4,".x","pos")
                pos_4 = namCheckandReturn(words,i,j,pos_4,".y","pos")
                pos_4 = namCheckandReturn(words,i,j,pos_4,".z","pos")
                pos_4 = namCheckandReturn(words,i,j,pos_4,".x","ori")
                pos_4 = namCheckandReturn(words,i,j,pos_4,".y","ori")
                pos_4 = namCheckandReturn(words,i,j,pos_4,".z","ori")
                pos_4 = namCheckandReturn(words,i,j,pos_4,".w","ori")
    f.close()

    return pos_1, pos_2, pos_3, pos_4, pos_5, pos_6

class BaseMovement(object):
    
    def __init__(self, file):
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_arm = baxter_interface.limb.Limb("left")
        self.pos = file
        # control parameters
        self._rate = 500.0  # Hz

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
        self.set_neutral()
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
    
    def moveToCoords(self, armName, coords):
        arm = Pose()
        arm.position.x = float(coords[1])
        arm.position.y = float(coords[3])
        arm.position.z = float(coords[5])
        arm.orientation.x = float(coords[7])
        arm.orientation.y = float(coords[9])
        arm.orientation.z = float(coords[11])
        arm.orientation.w = float(coords[13])

        joint_angles = self._ik_request(armName,arm)
        if joint_angles:
            if(armName == "left"):
                self._left_arm.move_to_joint_positions(joint_angles)
            elif(armName == "right"):
                self._right_arm.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided.")
 
    def abovePosition(self):
        self.set_neutral()
        rate = rospy.Rate(5)


        while not rospy.is_shutdown():
            self._pub_rate.publish(self._rate)
            p1,p2,p3,p4,p5,p6 = read(self.pos)

            com = 0
            com = input("position: ")

            if(com == '1' and len(com) < 2):
                self.moveToCoords('left',p1)
            elif(com == '2' and len(com) < 2):
                self.moveToCoords('right',p2)
            elif(com == '3' and len(com) < 2):
                self.moveToCoords('right',p3)
            elif(com == '4' and len(com) < 2):
                self.moveToCoords('left',p4)
            elif(com == '5' and len(com) < 2):
                self.moveToCoords('right',p5)
            elif(com == '6' and len(com) < 2):
                self.moveToCoords('left',p6)
            elif(com == '0' and len(com) < 2):
                self.set_neutral()
            elif(len(com) > 1):
                print("")
            elif(len(com) > 1 and com == b'/x03'):
                print("Stopping")
            else:
                print("invald command")

            rate.sleep()
            

def main():
    """Basic Movment: Path Following

    Commands joints to move to coordinates that where predefined 
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        "-f", "--file", required = True, 
        help = "Provide a file to read form"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("basic_movement_to_position")

    move = BaseMovement(args.file)
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)
    if(exiting != True):
        move.abovePosition()

    print("Done.")
    return 0

if __name__ == '__main__':
    main()
