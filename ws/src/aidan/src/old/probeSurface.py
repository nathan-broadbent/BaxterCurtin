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

from baxter_core_msgs.msg import (
    EndpointState
)

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
)

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

from aidan.msg import AtiMsg

#Costants Definitions
PI = np.pi                            

class ArmVelocity(object):
    def __init__(self):
            self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                            UInt16, queue_size=10)
            self._right_arm = baxter_interface.limb.Limb("right")
            self._left_arm = baxter_interface.limb.Limb("left")
            # control parameters
            self._rate = 250.0  # Hz

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
        # names = self._right_arm.joint_names()
        # jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        # angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
        #         jald["right_e1"], jald["right_w0"], jald["right_w1"],
        #         jald["right_w2"]]
        
        # pose = baxter_model.fkine(angles)
        # pnt = pose.t
        # pnt[2] = pnt[2]+0.1
        # self.optimToPoint(pnt)

        #Set Joint Velocities to 0
        # vel_dict = dict()
        # for idx, name in enumerate(names):
        #     vel_dict[name] = 0
        
        # self._right_arm.set_joint_velocities(vel_dict)

    def clean_shutdown(self):
        print("\nExiting...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def getPos(self):
        baxter_model = rtb.models.DH.Baxter('right')
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]

        print(f"Current Position: {baxter_model.fkine(angles)}\n\n")
    
    def setVelocityZero(self):
        names = self._right_arm.joint_names()
        vel_dict = dict()

        for idx, name in enumerate(names):
            vel_dict[name] = 0
        self._right_arm.set_joint_velocities(vel_dict)

    def optimToPoint(self, pos, rot=None, lin_vmax=0.02):
        global probing

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
        while (not arrived) and probing:
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
    global fz_av
    global calc_av_force
    global count
    global sum
    global probing
    # global contact
    
    size = 100
    threashold = 0.05
    
    if(calc_av_force): #For Calculating the Average Force
        sum+=data.fz
        count = count + 1
        # print(sum)
        if count == (size-1):
            fz_av = sum/size
            calc_av_force = False #stop loop from continuing
            count = 0
            sum = 0
            # print(fz_av)
    if(probing):
        if data.fz > (fz_av + threashold):
            count = count + 1
            if count > 20:
                print(f"Force at zHeight: {round(data.fz,5)}")
                print("Greater than threashold")
                probing = False
                count = 0

        

    #Interested in Fz
    # print(data.fx)
    # average_force = 0;

def endpointCallback(data):
    global zHeight
    zHeight = data.pose.position.z

def main():

    global calc_av_force #Should the average force in z be calculated
    global fz_av #default value for force in z with just gravity
    global count
    global sum
    global probing
    global zHeight

    count = 0
    sum = 0
    fz_av = 0
    calc_av_force = False #Trigger an Average Force Calculation
    probing = False

    print("Initializing node... ")
    rospy.init_node("arm_velocity_controller")

    move = ArmVelocity()
    exiting = move.clean_shutdown
    
    rospy.on_shutdown(exiting)
    r = rospy.Rate(1)
    print("In Function after")

    #initialise Other Subscriber and Publishers
    rospy.Subscriber("/atiForces", AtiMsg, atiCallback)
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, endpointCallback)


    if(exiting != True):
        print("In Main Loop")
        calc_av_force = True
    while(fz_av == 0): #Would Be Better To Do This With An Action Server
        r.sleep()  
    print(fz_av)
    probing = True #Start Probing Action
    while(probing):
        move.optimToPoint([0.8,-0.13,0.05], [1.485,0,1.571],0.01)
    move.setVelocityZero()
    print(f"zHeight Calculated: {round(zHeight,5)}")
    return(zHeight)
    
    
        
     
    # for _ in range(10000):
    #   print("hello")
    #   #Perform Probing Function
    #   #First Get Force Data in the z direction of ATI sensor
    #   #Establish a normal reading value
    #   #Descend at a slow velocity, until force value changes
    #   #record z value and repeat

    #   #assume Starts Above piece for probing

    #   #establish force normal for z
    #   for i in range(3):
    #     detected = False
    #     while not detected:
    #         print("Slowly Descending")
    #         move.optimisationDirWld([0,0,-0.05])
    #         #check for force here
        
    # print("Done.")    
    # return 0

if __name__ == '__main__':
    main()
