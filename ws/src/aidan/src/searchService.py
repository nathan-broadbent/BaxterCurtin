#!/usr/bin/env python3

import argparse
import rospy
import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import time
import math
from scipy.spatial.transform import Rotation as R
from pydrake.solvers import MathematicalProgram, Solve

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
from aidan.msg import AtiMsg

from aidan.srv import (
    searchHole
)

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

#Costants Definitions
PI = np.pi

                             

class ArmVelocity(object):
    def __init__(self):
            self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                            UInt16, queue_size=10)
            self._right_arm = baxter_interface.limb.Limb("right")
            self._left_arm = baxter_interface.limb.Limb("left")
            # control parameters
            self._rate = 250  # Hz

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
        # self._right_arm.move_to_neutral()
        # self._left_arm.move_to_neutral()

        # baxter_model = rtb.models.DH.Baxter('right')
        names = self._right_arm.joint_names()
        # jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        # angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
        #         jald["right_e1"], jald["right_w0"], jald["right_w1"],
        #         jald["right_w2"]]
        
        # pose = baxter_model.fkine(angles)
        # pnt = pose.t
        # pnt[2] = pnt[2]+0.1
        # self.optimToPoint(pnt)

        #Set Joint Velocities to 0
        vel_dict = dict()
        for idx, name in enumerate(names):
            vel_dict[name] = 0
        
        self._right_arm.set_joint_velocities(vel_dict)

    def clean_shutdown(self):
        print("\nExiting...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True
    

    def optimToPoint(self, pos, rot=None, lin_vmax=0.02):
        global searching

        joint_vmax = 1.0 #Max Angular Velocity for Any Joint
        # lin_vmax = 0.02 #Max lin Velocity 
        
        baxter_model = rtb.models.DH.Baxter('right')
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
        
        names = self._right_arm.joint_names() #Get Joint Names in Correct Order

        arrived = False #Storing if motion is complete
        gain = np.array([1,1,1,1,1,1]) #speed multipler * error
                                                    #(x,y,z)
        Te = baxter_model.fkine(angles).A #End-Effector pose (Get a numpy array)
        Tep = np.copy(Te)
        
        if(rot == None):
            Tep[0][3] = pos[0] #Change End Possition
            Tep[1][3] = pos[1]
            Tep[2][3] = pos[2]
        else:
            # r = R.from_quat(rot)
            # euler = r.as_euler("zyz")
            Tep = sm.SE3(pos[0], pos[1], pos[2]) * sm.SE3.RPY(rot[0], rot[1], rot[2])


        print(f"Goal Position: {Tep}\n\n")
        print(f"Current Position: {Te}\n\n")
        
        i = 0
        while (not arrived) and searching:
            jald = self._right_arm.joint_angles() #Gets joint angles of right arm
            angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                        jald["right_e1"], jald["right_w0"], jald["right_w1"],
                        jald["right_w2"]]
            Te = baxter_model.fkine(angles).A #End-Effector pose (Get a numpy array)
            
            
            ev, arrived_ = rtb.p_servo(Te, Tep, gain=gain, threshold=0.005, method='angle-axis')
            vel = np.linalg.norm(ev[:3])
            if (vel > lin_vmax):
                # print(f"Before: {ev[:3]}")
                ev[:3] = np.dot(lin_vmax/vel, ev[:3])
                # print(f"After: {ev[:3]}")


            v_des = ev
            J = baxter_model.jacob0(angles)
            prog = MathematicalProgram()
            v = prog.NewContinuousVariables(7, "v")
            
        

            error = J @ v - v_des
            prog.AddCost(error.dot(error))
            prog.AddBoundingBoxConstraint(-joint_vmax, joint_vmax, v)
            result = Solve(prog)
            joint_vel = result.GetSolution(v)
            
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = joint_vel[idx]
            self._right_arm.set_joint_velocities(vel_dict)

            #Calc If Exiting Condition is Met
            pos_e = np.linalg.norm([pos[0]-Te[0][3], pos[1]-Te[1][3], pos[2]-Te[2][3]])
            if(pos_e < 0.002):
                arrived = True

            if i % 1000 == 0:
                # print(f"Pos Error: {[Tep[0][3]-pos[0], Tep[1][3] - pos[1], Tep[2][3]-pos[2]]}")
                # print(type(Tep))
                # print(f"{joint_vel}")
                print(f"POS Error: {round(pos_e,5)}")
                # print(f"v description: {v_des}")
                print(f"Velocity: {round(vel,4)}, Rectified Vel: {round(np.linalg.norm(ev[:3]),3)}")
                # print(f"{(Tep-Te)}\n\n")
            i = i+1
        if arrived:
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = 0
            self._right_arm.set_joint_velocities(vel_dict)
            # return result.is_success(), result.GetSolution(v)

def atiCallback(data):
    global count
    global searching
    global fy
    global firstExecution
    # global contact
    
    size = 100
    threashold = fy + 0.05
    if(firstExecution):
        fy = data.fy
        print(f"Current Fy: {fy}")
        firstExecution = False
    if(data.fy < threashold - 0.15):
        threashold = threashold - 0.001
        
    
    if(searching):
        # print(data.fy)
        if data.fy > (threashold):
            count = count + 1
            if count > 10:
                print(data.fy)
                print("Greater than threashold")
                searching = False
                count = 0


def main(zheight):
    global searching
    global count

    count = 0
    searching = False
    
    print("Initializing node... ")
    rospy.init_node("search_controller")
    rospy.Subscriber("/atiForces", AtiMsg, atiCallback)
    
    move = ArmVelocity()
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)
    print("In Function after")
    print(f"ZHeight: {zheight}")
    if(exiting != True):
        print("In Main Loop")

    

    searching = True  
    # for _ in range(10000):
    while(searching):
        # move.moveToWld([0.55,-0.6,0.25])
        # move.dambedsqrToPointWld([0.55,-0.6,0.25],np.array([-0.1459, 0.9889, -0.0086, 0.0233]))
        # move.optimToPoint([0.55,-0.6,0.25])
        length = 0.03
        width = 0.03
        div = 7
        center = [0.808,-0.135]
        center[0] = center[0]+0.035 #Gripper Offset
        # offset = 0.05
        if searching:
            for i in range(div):
                move.optimToPoint([(center[0]-width/2 + (width/(div-1)*i)), (center[1]-length/2),zheight + 0.005],[1.485,0.2,1.571], 0.05)
                move.optimToPoint([(center[0]-width/2 + (width/(div-1)*i)), (center[1]-length/2),zheight],[1.485,0.2,1.571])
                move.optimToPoint([(center[0]-width/2 + (width/(div-1)*i)), (center[1]+length/2),zheight],[1.485,0.2,1.571])
                
        print("Optimisation cycle complete")    
        
    print("Done.")
    return 0

def handle_search_hole(req):
    print("Search For Hole Called!")
    global move
    global searching
    global count
    global firstExecution
    
    firstExecution = True #Calc Current force in the Y Direction
    count = 0
    searching = False
    

    center = [req.xPos, req.yPos]
    center[0] = center[0]+0.035 #Gripper Offset
    zheight = req.zHeight
    width = req.searchWidth
    length = req.searchLength

    searching = True  #Start Search Opperation
    repeats = 2
    rep = 0
    # for _ in range(10000):
    print(f"Center: {center}")
    print(f"width: {width}")
    print(f"length: {length}")
    while(searching and rep < repeats):
        div = 4
        if searching and rep < repeats:
            for i in range(div):
                move.optimToPoint([(center[0]-width/2 + (width/(div-1)*i)), (center[1]-length/2),zheight + 0.005],[1.485,0.2,1.571], 0.05)
                move.optimToPoint([(center[0]-width/2 + (width/(div-1)*i)), (center[1]-length/2),zheight],[1.485,0.2,1.571])
                move.optimToPoint([(center[0]-width/2 + (width/(div-1)*i)), (center[1]+length/2),zheight],[1.485,0.2,1.571])
        rep = rep + 1 
        zheight -= 0.005
    success = not searching
    searching = False  
    return success
          

def searchService():
    print("Search Service Starting")
    global move
    global searching
    global count
    global fy
    global firstExecution

    count = 0
    searching = False
    fy = 0.35
    firstExecution = False
    
    
    print("Initializing node... ")
    rospy.init_node("search_server")
    rospy.Subscriber("/atiForces", AtiMsg, atiCallback)
    s = rospy.Service("search_hole", searchHole, handle_search_hole)
    
    move = ArmVelocity()
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)

    if(exiting != True):
        print("Wating For Probe Command")
        rospy.spin()

if __name__ == '__main__':
    searchService()
    # zhight = 0.18
    # print(zhight)
    # main(zhight)
