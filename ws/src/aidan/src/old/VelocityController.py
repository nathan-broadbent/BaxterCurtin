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

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

#Costants Definitions
PI = np.pi

#Distance Offsets (m)
L0 = 0.27035
L1 = 0.069
L2 = 0.36435
L3 = 0.069
L4 = 0.37429
L5 = 0.01
L6 = 0.3945

FINAL_TRANS_LEFT = np.array([[np.cos(PI/4), -np.sin(PI/4), 0, 0],
                             [np.sin(PI/4), np.cos(PI/4), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
FINAL_TRANS_RIGHT = np.array([[np.cos(-PI/4), -np.sin(-PI/4), 0, 0],
                             [np.sin(-PI/4), np.cos(-PI/4), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])

FINAL_TRANS_LEFT_R = np.array([[np.cos(PI/4), -np.sin(PI/4), 0],
                             [np.sin(PI/4), np.cos(PI/4), 0],
                             [0, 0, 1]])
                             

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
    
    def vel_ctr(self, goal, vel): #Does Not as Well Function Properly

        baxter_model = rtb.DHRobot(
            [
                # rtb.RevoluteDH(),
                rtb.RevoluteDH(a=L1, alpha=PI/2, d=L0, offset=PI/4),
                rtb.RevoluteDH(alpha=PI/2, offset=PI/2),
                rtb.RevoluteDH(a=L3, alpha=-PI/2, d=L2),
                rtb.RevoluteDH(alpha=PI/2),
                rtb.RevoluteDH(a=L5, alpha=-PI/2, d=L4),
                rtb.RevoluteDH(alpha=PI/2),
                rtb.RevoluteDH(d=L6)
            ]
        )
        error = 0.5
        iter = 0
        while(error >= 0.05):
            jald = self._left_arm.joint_angles() #Gets joint angles of left arm
            angles = [jald["left_s0"], jald["left_s1"], jald["left_e0"], 
                        jald["left_e1"], jald["left_w0"], jald["left_w1"],
                        jald["left_w2"]]
            cur_trans = baxter_model.fkine(angles)
            # print(cur_trans.t)
            # print(cur_trans)
            # print(type(cur_trans))
            # cur_pos = [cur_trans[0][3], cur_trans[1][3], cur_trans[2][3]]
            cur_pos = cur_trans.t
            # cur_pos = np.matmul(FINAL_TRANS_LEFT_R, cur_pos)
            print(f"POS: {cur_pos}")
            print(f"Error: {error}")
            diff = [goal[0] - cur_pos[0], goal[1] - cur_pos[1], goal[2] - cur_pos[2]]
            error = np.sum(np.square(diff))
            try:
                vel_x = ((diff[0]**2 / error)**0.5)*vel
                vel_y = ((diff[1]**2 / error)**0.5)*vel
                vel_z = ((diff[2]**2 / error)**0.5)*vel
            except TypeError as e:
                vel_x = 0
                vel_y = 0
                vel_z = 0
                print(e)
            # out_vel = np.matmul(FINAL_TRANS_LEFT_R, [vel_x, vel_y, vel_z])
            # out_vel = [out_vel[0], out_vel[1], out_vel[2], 0, 0, 0]

            # print(f"current  vel: {[vel_x, vel_y, vel_z]}, Trans: {out_vel}")
            out_vel = [vel_x, vel_y, vel_z, 0, 0, 0]
            J = baxter_model.jacob0(angles)
            # print(J)

            Jinv = np.linalg.pinv(J)
            joint_vel = Jinv @ out_vel
            # print(joint_vel)
            # print(f"test {iter}")
            iter += 1

            names = self._left_arm.joint_names()
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = joint_vel[idx]
            
            # print(vel_dict)
            #Set Joint Velocities
            self._left_arm.set_joint_velocities(vel_dict)
            
            # print(cur_pos)

    def getPos(self):
        baxter_model = rtb.DHRobot(
            [
                # rtb.RevoluteDH(),
                rtb.RevoluteDH(a=L1, alpha=PI/2, d=L0, offset=PI/4),
                rtb.RevoluteDH(alpha=PI/2, offset=PI/2),
                rtb.RevoluteDH(a=L3, alpha=-PI/2, d=L2),
                rtb.RevoluteDH(alpha=PI/2),
                rtb.RevoluteDH(a=L5, alpha=-PI/2, d=L4),
                rtb.RevoluteDH(alpha=PI/2),
                rtb.RevoluteDH(d=L6)
            ]
        )
        jald = self._left_arm.joint_angles() #Gets joint angles of left arm
        angles = [jald["left_s0"], jald["left_s1"], jald["left_e0"], 
                    jald["left_e1"], jald["left_w0"], jald["left_w1"],
                    jald["left_w2"]]
        cur_trans = baxter_model.fkine(angles)
        cur_pos = cur_trans.t
        # cur_pos = np.matmul(FINAL_TRANS_LEFT_R, cur_pos)
        print(f"POS: {cur_pos}")
        # print(f"Error: {error}")

    def moveVel(self, vel):

        # baxter_model = rtb.DHRobot(
        #     [
        #         # rtb.RevoluteDH(),
        #         rtb.RevoluteDH(a=L1, alpha=PI/2, d=L0, offset=PI/4),
        #         rtb.RevoluteDH(alpha=PI/2, offset=PI/2),
        #         rtb.RevoluteDH(a=L3, alpha=-PI/2, d=L2),
        #         rtb.RevoluteDH(alpha=PI/2),
        #         rtb.RevoluteDH(a=L5, alpha=-PI/2, d=L4),
        #         rtb.RevoluteDH(alpha=PI/2),
        #         rtb.RevoluteDH(d=L6)
        #     ]
        # )
        baxter_model = rtb.models.DH.Baxter('left')
        while(self.clean_shutdown != True):
            jald = self._left_arm.joint_angles() #Gets joint angles of left arm
            angles = [jald["left_s0"], jald["left_s1"], jald["left_e0"], 
                        jald["left_e1"], jald["left_w0"], jald["left_w1"],
                        jald["left_w2"]]
            cur_trans = baxter_model.fkine(angles) #Calculate World -> end effector Forward Kinematic Transformation Matrix
            cur_pos = cur_trans.t
            J = baxter_model.jacob0(angles) #Get Jocobian For Current Joint Config
                # print(J)

            Jinv = np.linalg.pinv(J) #Calc Pseudo Inverse of Jacobian
            joint_vel = np.matmul(Jinv, vel) #Calc Required Joint Velocities
            names = self._left_arm.joint_names()
            # print(names)
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = joint_vel[idx]
            
            # print(vel_dict)
            #Set Joint Velocities
            self._left_arm.set_joint_velocities(vel_dict)
            # print(f"Joint Vel: {joint_vel}")
            # print(f"POS: {cur_pos}")

    def moveDirEndEff(self, trans): #reccommended Velocity Control
        baxter_model = rtb.models.DH.Baxter('right')
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
        
        names = self._right_arm.joint_names() #Get Joint Names in Correct Order

        baxter_model.q = angles #Set rtb model angles to baxters angles
        arrived = False #Storing if motion is complete
        gain = np.array([0.8,0.8,0.8,0.3,0.3,0.3]) #speed multipler * error
                                                    #(x,y,z)
        Tep = baxter_model.fkine(baxter_model.q) * sm.SE3.Trans(0,0.1,0) #* sm.SE3.RPY(0,0,0) #goal pos and rot
        Tep = Tep.A

        print(f"Goal Position: {Tep}\n\n")
        print(f"Current Position: {baxter_model.fkine(baxter_model.q)}\n\n")

        i = 0
        while not arrived:
        # for i in range(10000):
            if self.clean_shutdown == True:
                break

            #Getting Joint Angles From The Robot
            jald = self._right_arm.joint_angles() #Gets joint angles of right arm
            angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
            baxter_model.q = angles

            J = baxter_model.jacobe(baxter_model.q) #Work out base frame Jacobian Manipulator
            
            Te = baxter_model.fkine(baxter_model.q).A #End-Effector pose (Get a numpy array)

            J_pinv = np.linalg.pinv(J)

            #calc required end effector velocity and if it was arrived
            ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.02, method='rpy')
            baxter_model.qd = J_pinv @ ev

            #applying Velocity to actual robotArm
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = baxter_model.qd[idx]
            
            self._right_arm.set_joint_velocities(vel_dict)

            if i % 1000 == 0:
                print(f"{baxter_model.qd}")
                print(f"{Tep-Te}\n\n")
            i = i+1
        if arrived:
            print("Successfully Reached Possition")

    def moveDirWld(self, pos): #Reccommended Velocity Control
        baxter_model = rtb.models.DH.Baxter('right')
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
        
        names = self._right_arm.joint_names() #Get Joint Names in Correct Order

        baxter_model.q = angles #Set rtb model angles to baxters angles
        arrived = False #Storing if motion is complete
        gain = np.array([0.8,0.8,0.8,0.3,0.3,0.3]) #speed multipler * error
                                                    #(x,y,z)
        Tep = baxter_model.fkine(baxter_model.q)
        Tep = Tep.A
        Tep[0][3] = Tep[0][3] + pos[0]
        Tep[1][3] = Tep[1][3] + pos[1]
        Tep[2][3] = Tep[2][3] + pos[2]

        print(f"Goal Position: {Tep}\n\n")
        print(f"Current Position: {baxter_model.fkine(baxter_model.q)}\n\n")

        i = 0
        while not arrived and self.clean_shutdown != True:
        # for i in range(10000):
            if self.clean_shutdown == True:
                break

            #Getting Joint Angles From The Robot
            jald = self._right_arm.joint_angles() #Gets joint angles of right arm
            angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
            baxter_model.q = angles

            J = baxter_model.jacob0(baxter_model.q) #Work out base frame Jacobian Manipulator
            
            Te = baxter_model.fkine(baxter_model.q).A #End-Effector pose (Get a numpy array)

            J_pinv = np.linalg.pinv(J)

            #calc required end effector velocity and if it was arrived
            ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.02, method='angle-axis')
            baxter_model.qd = J_pinv @ ev

            #applying Velocity to actual robotArm
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = baxter_model.qd[idx]
            
            self._right_arm.set_joint_velocities(vel_dict)

            if i % 1000 == 0:
                print(f"{baxter_model.qd}")
                print(f"{Tep-Te}\n\n")
            i = i+1
        if arrived:
            print("Successfully Reached Possition")

    def moveToWld(self, pos): #Reccommended Velocity Control
        baxter_model = rtb.models.DH.Baxter('right')
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
        
        names = self._right_arm.joint_names() #Get Joint Names in Correct Order

        baxter_model.q = angles #Set rtb model angles to baxters angles
        arrived = False #Storing if motion is complete
        gain = np.array([0.2,0.2,0.2,0.3,0.3,0.3]) #speed multipler * error
                                                    #(x,y,z)
        Tep = baxter_model.fkine(baxter_model.q)
        Tep = Tep.A
        Tep[0][3] = pos[0]
        Tep[1][3] = pos[1]
        Tep[2][3] = pos[2]

        print(f"Goal Position: {Tep}\n\n")
        print(f"Current Position: {baxter_model.fkine(baxter_model.q)}\n\n")

        i = 0
        while not arrived and self.clean_shutdown != True:
        # for i in range(10000):
            if self.clean_shutdown == True:
                break

            #Getting Joint Angles From The Robot
            jald = self._right_arm.joint_angles() #Gets joint angles of right arm
            angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
            baxter_model.q = angles

            J = baxter_model.jacob0(baxter_model.q) #Work out base frame Jacobian Manipulator
            
            Te = baxter_model.fkine(baxter_model.q).A #End-Effector pose (Get a numpy array)

            J_pinv = np.linalg.pinv(J)

            #calc required end effector velocity and if it was arrived
            ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.02, method='angle-axis')
            baxter_model.qd = J_pinv @ ev

            #applying Velocity to actual robotArm
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = baxter_model.qd[idx]
            
            self._right_arm.set_joint_velocities(vel_dict)

            if i % 1000 == 0:
                print(f"{baxter_model.qd}")
                print(f"{Tep-Te}\n\n")
            i = i+1
        if arrived:
            print("Successfully Reached Possition")

    def getPos(self):
        baxter_model = rtb.models.DH.Baxter('right')
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]

        print(f"Current Position: {baxter_model.fkine(angles)}\n\n")

    def optimToPoint(self, pos, rot=None):
        global searching

        joint_vmax = 1.0 #Max Angular Velocity for Any Joint
        lin_vmax = 0.02 #Max lin Velocity 
        
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
                print(f"Error: {pos_e}")
                # print(f"v description: {v_des}")
                print(f"Velocity: {vel}")
                # print(f"{(Tep-Te)}\n\n")
            i = i+1
        if arrived:
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = 0
            self._right_arm.set_joint_velocities(vel_dict)
            # return result.is_success(), result.GetSolution(v)
    
    def optimisationDirWld(self, lin_v, ang_v):
        robot = rtb.models.DH.Baxter('right')
        jald = self._left_arm.joint_angles() #Gets joint angles of left arm
        q = [jald["left_s0"], jald["left_s1"], jald["left_e0"], 
                    jald["left_e1"], jald["left_w0"], jald["left_w1"],
                    jald["left_w2"]]
        names = self._right_arm.joint_names() #Get Joint Names in Correct Order

        J = robot.jacob0(q)
        prog = MathematicalProgram()
        v = prog.NewContinuousVariables(7, "v")
        vmax = 0.05
        v_des = np.zeros(6)
        v_des[:3] = lin_v
        v_des[3:] = ang_v
        # print(J)
        # print(v)
        error = J @ v - v_des
        prog.AddCost(error.dot(error))
        prog.AddBoundingBoxConstraint(-vmax, vmax, v)
        result = Solve(prog)
        joint_vel = result.GetSolution(v)

        vel_dict = dict()
        for idx, name in enumerate(names):
            vel_dict[name] = joint_vel[idx]
        self._right_arm.set_joint_velocities(vel_dict)

        return result.is_success(), result.GetSolution(v)

    def dambedsqrToPointWld(self, pos_d, quat_d):
        iters = 1000
        tol = 0.005
        damp = 1
        success = False

        baxter_model = rtb.models.DH.Baxter('right')
        # jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        # angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
        #         jald["right_e1"], jald["right_w0"], jald["right_w1"],
        #         jald["right_w2"]]
        # pose = baxter_model.fkine(angles)
        # rpy = pose.rpy()
        # quat_d = R.from_euler('xyz', rpy).as_quat()
        print(quat_d)
        names = self._right_arm.joint_names()
        
        # for _ in range(iters):
        while not success:
            jald = self._right_arm.joint_angles() #Gets joint angles of right arm
            angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
            
            pose = baxter_model.fkine(angles)
            rpy = pose.rpy()
            t = pose.t
            quat = R.from_euler('xyz', rpy).as_quat()
            error = np.zeros(6)
            error[:3] = pos_d - t
            orient_error = quat[0] * quat_d[1:] - quat_d[0] * quat[1:] - np.cross(quat_d[1:], quat[1:])
            error[3:] = orient_error
            
            if np.linalg.norm(error) < tol:
                break
            J = baxter_model.jacob0(angles)
            JT = np.transpose(J)
            dq = JT @ np.linalg.pinv(np.add(J @ JT, (damp**2) * np.eye(6))) @ error
            # print(dq)

            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = dq[idx]
            
            self._right_arm.set_joint_velocities(vel_dict)



# def atiCallback(data):
#     global count
#     global searching
#     # global contact
    
#     size = 100
#     threashold = 0.33
    
#     if(searching):
#         # print(data.fy)
#         if data.fy > (threashold):
#             count = count + 1
#             if count > 10:
#                 print(data.fy)
#                 print("Greater than threashold")
#                 searching = False
#                 count = 0


def main(zheight):
    global searching
    global count

    count = 0
    searching = False
    
    print("Initializing node... ")
    # rospy.init_node("search_controller")
    # rospy.Subscriber("/atiForces", AtiMsg, atiCallback)
    
    move = ArmVelocity()
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)
    print("In Function after")
    if(exiting != True):
        print("In Main Loop")

    

    searching = True  
    # for _ in range(10000):
    while(searching):
        # move.moveToWld([0.55,-0.6,0.25])
        # move.dambedsqrToPointWld([0.55,-0.6,0.25],np.array([-0.1459, 0.9889, -0.0086, 0.0233]))
        # move.optimToPoint([0.55,-0.6,0.25])
        length = 0.2
        width = 0.1
        div = 7
        offset = 0.05
        if searching:
            for i in range(div):
                # move.optimToPoint([0.80 + (width/(div-1)*i), -0.2, zhight], [1.485,0.2,1.571])
                move.optimToPoint([0.80 + (width/(div-1)*i), -0.23+offset, zheight], [1.485,0.2,1.571])
                move.optimToPoint([0.80 + (width/(div-1)*i), -0.05, zheight], [1.485,0.2,1.571])
                move.optimToPoint([0.80 + (width/(div-1)*i), -0.05-offset, zheight+0.01], [1.485,-0.2,1.571])
                move.optimToPoint([0.80 + (width/(div-1)*i), -0.05-offset, zheight], [1.485,-0.2,1.571])
                move.optimToPoint([0.80 + (width/(div-1)*i), -0.23, zheight], [1.485,-0.2,1.571])
                move.optimToPoint([0.80 + (width/(div-1)*i), -0.23+offset, zheight+0.01], [1.485,-0.2,1.571])
        # move.optimToPoint([0.75,-0.1,0.2],[1.485,0,1.571])
        # move.optimToPoint([0.9,-0.1,0.2],[1.485,0,1.571])
        # move.optimToPoint([0.9,0.1,0.2],[1.485,0,1.571])
        # move.optimToPoint([0.75,0.1,0.2],[1.485,0,1.571])
        # move.optimToPoint([0.75,-0.1,0.2],[1.485,0,1.571])
        # move.optimToPoint([0.572,0,0.209])
        # move.optimToPoint([0.572,-0.8,0.209])
        # move.optimToPoint([0.7,-0.2,0.2], [-0.478, -0.478, -0.521, -0.521 ])
        # move.optimToPoint([0.7,-0.2,0.2])
        # move.getPos()
        print("Optimisation cycle complete")
        # rospy.sleep(2)

        # _, res = move.optimisationDirWld([0.05,0,0], [0,0,0])
        # move.moveDirWld([0,0.05,0])
        # move.dambedsqrToPointWld([0.572,-0.181,0.209],np.array([-0.38124197, -0.92309993,  0.01931754, -0.04656096]))

        # move.optimToPoint([0.8, -0.1, 0.1])
        # rospy.sleep(10)
        # print(res)

        # div = 4
        # lenth = 0.2 #y
        # depth = 0.2 #x
        # x = 0.7
        # y = -0.2
        # z = 0.2
        # xstep = depth/(div-1)
        # move.moveDirWld([x,y,z+0.05])
        # for _ in range(10):
        #     move.moveDirWld([x,y,z])
        #     for i in range(div):
        #         move.moveToWld([x+xstep*i,y,z])
        #         move.moveToWld([x+xstep*i,y+lenth,z])
        #         move.moveToWld([x+xstep*i,y,z])
        #     move.moveDirWld([x,y,z+0.05])
        #     time.sleep(1000)


            # move.getPos()
            # time.sleep(1000)
            # move.moveDirWld([0,0.1,0])
            # move.moveDirWld([0.03,0,0])
            # move.moveDirWld([0,-0.1,0])
            # move.moveDirWld([0.03,0,0])
            # move.moveDirWld([0,0.1,0])
            # move.moveDirWld([0.05,0,0])
            # move.moveDirWld([0,-0.2,0])
            # move.moveDirWld([-0.10,0,0])
            # move.moveDirWld([-0.05,0,0])
            # move.moveDirWld([0, 0.15,0])
            # move.moveDirWld([-0.07, 0.07,0])
            # move.moveDirWld([-0.07, -0.07,0])
            # move.moveDirWld([0, -0.15,0])
            # move.moveDirWld([0.07, -0.07,0])
            # move.moveDirWld([-0.07, 0.07,0])
            
            
       
        
        
        # print(FINAL_TRANS_LEFT)
        
    print("Done.")
    return 0

if __name__ == '__main__':
    zhight = 0.137
    main(zhight)
