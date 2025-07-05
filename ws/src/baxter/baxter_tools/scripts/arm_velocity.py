#!/usr/bin/env python3

import argparse
import rospy
import numpy as np
import roboticstoolbox as rtb

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

    def create_dh_array(self, joints):
        dh = np.array([[L1, -PI/2, joints[0], L0],      #S0
                       [0, PI/2, joints[1] + PI/2, 0],  #S1
                       [L3, -PI/2, joints[2], L2],      #E0
                       [0, PI/2, joints[3], 0],         #E1
                       [L5, -PI/2, joints[4], L4],      #W0
                       [0, PI/2, joints[5], 0],         #W1
                       [0, 0, joints[6], L6]])          #W2
        return dh

    def trans_multi(self, dh_array, start, finish, arm): #Calc H Transforms between base and joints
        h_matrixs = []
        #Deal with initial transform
        if arm == "left":
            trans = FINAL_TRANS_LEFT
        elif arm == "right":
            trans = FINAL_TRANS_RIGHT
        else:
            ValueError(f"Arm name not correct: {arm}")
        # trans = np.identity(4)
        print(trans)
        for i in range(start, finish):
            temp = self.trans_sing(dh_array, i)
            print(temp)
            trans = np.matmul(trans, temp)  #Error Here
            # trans = trans @ temp
            print(f"Inter {i}: \n{trans}")
            trans = np.round(trans, decimals=20)
            h_matrixs.append(trans)
        return h_matrixs
    
    def trans_sing(self, dh_array, idx): #Calc H Transforms Between Joints
        #Rotational values
        Cth = np.cos(dh_array[idx][2])
        Sth = np.sin(dh_array[idx][2])
        Ca = np.cos(dh_array[idx][1])
        Sa = np.cos(dh_array[idx][1])

        #Offset Values (m)
        d = dh_array[idx][3]
        r = dh_array[idx][0]

        trans = np.array([[Cth, -Sth*Ca, Sth*Sa, r*Cth],
                          [Sth, Cth*Ca, -Cth*Sa, r*Sth],
                          [0, Sa, Ca, d],
                          [0, 0, 0, 1]])
        # print(trans)
        return trans
    

    def get_jacobian(self, h_matrixs):
        J = []
        #Note need to apply base transformation
        #Get Displacement [x, y, z]
        D07 = [h_matrixs[len(h_matrixs) - 1][0][3], 
               h_matrixs[len(h_matrixs) - 1][1][3], 
               h_matrixs[len(h_matrixs) - 1][2][3]]
        
        #Calc base case
        rot = [0, 0, 1]
        lin = np.cross(rot, D07)
        J.append(np.append(lin, rot)) #First Column of Jacobian

        for i in range(len(h_matrixs)- 1): #Other 6 columns of Jacobian
            rot = [h_matrixs[i][0][2], h_matrixs[i][1][2], h_matrixs[i][2][2]]
            D0i = [h_matrixs[i][0][3], h_matrixs[i][1][3], h_matrixs[i][2][3]]
            lin = np.cross(rot, (np.subtract(D07, D0i)))
            J.append(np.append(lin, rot))
        J = np.transpose(J)

        return J
    
    def calc_joint_velocity(self, goal, vel, arm):
        #Get Current Arm Joint Angles & Convert joint angles to array
        error = 1
        itera = 0
        while (error**0.5 > 0.05 and itera < 1000): #Stop movement if within 5 cm
            if(arm == "left"):
                names = self._left_arm.joint_names()
                # names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
                jald = self._left_arm.joint_angles() #Gets joint angles of left arm
                angles = [jald["left_s0"], jald["left_s1"], jald["left_e0"], 
                            jald["left_e1"], jald["left_w0"], jald["left_w1"],
                            jald["left_w2"]]
            elif(arm == "right"):
                names = self._right_arm.joint_names()
                jard = self._right_arm.joint_angles() #Gets joint angles of right arm
                angles = [jard["right_s0"], jard["right_s1"], jard["right_e0"], 
                            jard["right_e1"], jard["right_w0"], jard["right_w1"],
                            jard["right_w2"]]
            else:
                ValueError(f"Arm Names is not correct: {arm}")
            # print(angles_left)
            # print(angles_right)

            #Calc required velocity
            dh = self.create_dh_array(angles)
            h_trans = self.trans_multi(dh, 0, 7, arm)
            cur_pos = [h_trans[6][0][3], h_trans[6][1][3], h_trans[6][2][3]]
            diff = [goal[0] - cur_pos[0], goal[1] - cur_pos[1], goal[2] - cur_pos[2]]
            # diff = np.subtract(goal, cur_pos)
            error = np.sum(np.square(diff))
            try:
                vel_x = ((diff[0]**2 / error)**0.5)*vel
                vel_y = ((diff[1]**2 / error)**0.5)*vel
                vel_z = ((diff[2]**2 / error)**0.5)*vel
            except TypeError as e:
                vel_x = 0
                vel_y = 0
                vel_z = 0
                print(((diff[0]**2 / error)**0.5))
                # print(goal)
                print(e)

            out_vel = [vel_x, vel_y, vel_z, 0, 0, 0]

            j = self.get_jacobian(h_trans)
            j_inv = np.linalg.pinv(j)

            joint_vel = np.matmul(j_inv, out_vel)
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = joint_vel[idx]
            

            #Set Joint Velocities
            if(arm == "left"):
                self._left_arm.set_joint_velocities(vel_dict)
                # print(vel_dict)
                print(cur_pos)

            itera += 1

        print(itera)
        return joint_vel
    
    def vel_ctr(self, goal, vel):

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
            cur_trans = baxter_model.fkine(angles)
            cur_pos = cur_trans.t
            J = baxter_model.jacob0(angles)
                # print(J)

            Jinv = np.linalg.pinv(J)
            joint_vel = np.matmul(Jinv, vel)
            names = self._left_arm.joint_names()
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = joint_vel[idx]
            
            # print(vel_dict)
            #Set Joint Velocities
            self._left_arm.set_joint_velocities(vel_dict)
            # print(f"Joint Vel: {joint_vel}")
            # print(f"POS: {cur_pos}")
        



            



def main():

    print("Initializing node... ")
    rospy.init_node("arm_velocity_controller")

    move = ArmVelocity()
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)
    print("In Function after")
    if(exiting != True):
        print("In Main Loop")
        # move.getPos()
        # move.vel_ctr([0.3, 0.3, 0.5], 0.02)
        move.moveVel([0.0, -0.02, 0.0, 0, 0, 0])
        # jacob = move.calc_joint_velocity([-0.07, 0.51, -0.02], 0.01, "left")
        
        
        # print(FINAL_TRANS_LEFT)
        
    print("Done.")
    return 0

if __name__ == '__main__':
    main()
