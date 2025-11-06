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

import baxter_interface
import struct

from baxter_interface import CHECK_VERSION

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

from nathan.srv import (
    Probe
)

from aidan.msg import AtiMsg


class ArmVelocity(object):
    def __init__(self):
            self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                            UInt16, queue_size=10)
            self._right_arm = baxter_interface.limb.Limb("right")
            self._left_arm = baxter_interface.limb.Limb("left")
            
            self.baxter_model = rtb.models.DH.Baxter('right')
            self.joint_names = self._right_arm.joint_names()
 
            self.probing = False

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

    def clean_shutdown(self):
        print("\nExiting...")
        #return to normal
        self._reset_control_modes()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def _getJacobian(self, angles):
        return self.baxter_model.jacob0(angles)

    def _getPos(self, angles):        
        return self.baxter_model.fkine(angles).A

    def getPos(self):
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
        print(f"Angles: {angles}")
        
        return self._getPos(angles)
    
    def getJacobianAndPos(self):
        jald = self._right_arm.joint_angles() #Gets joint angles of right arm
        angles = [jald["right_s0"], jald["right_s1"], jald["right_e0"], 
                    jald["right_e1"], jald["right_w0"], jald["right_w1"],
                    jald["right_w2"]]
        return self._getJacobian(angles), self._getPos(angles)

    def setVelocityZero(self):
        
        vel_dict = dict()

        for idx, name in enumerate(self.joint_names):
            vel_dict[name] = 0
        self._right_arm.set_joint_velocities(vel_dict)

    def optimToPoint(self, pos, quat=None, lin_vmax=0.02):
        # Moves the end effector to a specified position using optimization to calculate joint velocities
        # Will stop if self.probing is set to False
        # will move in a straight line to the target position
        # will maintain current orientation unless a rotation is provided
        # pos: Target position as a list or array [x, y, z]
        # rot: Target rotation as a list or array quaternion [qx, qy, qz, qw]
        # lin_vmax: Maximum linear velocity for the end effector

        joint_vmax = 1.0 #Max Angular Velocity for Any Joint        
        # Get current end-effector pose
        Te = self.getPos() # End-Effector pose
        
        #(x,y,z)
        # Te - Current End-Effector Pose (Transformation Matrix)
        Tep = np.copy(Te)
        
        # Set Target position if the rotation is not provided
        if(quat is None):
            Tep[0][3] = pos[0] #Change End Possition
            Tep[1][3] = pos[1]
            Tep[2][3] = pos[2]
        else:
            # Convert quaternion to [w, qx, qy, qz]
            quat = [quat[3], quat[0], quat[1], quat[2]]
            print(f"Quaternion - rearanged: {quat}")
            
            # Calculate Target End-Effector Pose with Rotation
            rot = sm.UnitQuaternion(quat).R
            print(f"rotation_matrix {rot}")
            print(f"pos {pos}")
            Tep[:3, :3] = rot
            Tep[:3, 3] = pos

            Tep = sm.SE3(Tep)
            
            print(f"goal_len {len(Tep)}")

        print(f"Goal Position: {Tep}\n\n")
        print(f"Current Position: {Te}\n\n")
        
        arrived = False #Storing if motion is complete
        gain = np.array([1,1,1,1,1,1]) #speed multipler * error
        i = 0
        # While loop until the end-effector reaches the target position - or probing is stopped (by force feedback or other condition)
        while (not arrived) and self.probing:
            J, Te = self.getJacobianAndPos()
            
            ev, arrived_ = rtb.p_servo(Te, Tep, gain=gain, threshold=0.005, method='angle-axis')
            vel = np.linalg.norm(ev[:3])
            if (vel > lin_vmax):
                ev[:3] = np.dot(lin_vmax/vel, ev[:3])

            # Initialize Mathematical Program for optimization
            v_des = ev
            prog = MathematicalProgram()
            v = prog.NewContinuousVariables(7, "v")

            # Calculate joint velocities using jacobian transpose method
            error = J @ v - v_des
            prog.AddCost(error.dot(error))
            prog.AddBoundingBoxConstraint(-joint_vmax, joint_vmax, v)
            result = Solve(prog)
            joint_vel = result.GetSolution(v)

            # Apply joint velocities
            vel_dict = dict()
            for idx, name in enumerate(self.joint_names):
                vel_dict[name] = joint_vel[idx]
            self._right_arm.set_joint_velocities(vel_dict)

            # Calculate position error
            pos_e = np.linalg.norm([pos[0]-Te[0][3], pos[1]-Te[1][3], pos[2]-Te[2][3]])
            if(pos_e < 0.002): # Check if within 2mm of target
                arrived = True

            # Print status every 1000 iterations
            if i % 1000 == 0:
                print(f"POS Error: {round(pos_e,5)}")
                print(f"Velocity: {round(vel,4)}, Rectified Vel: {round(np.linalg.norm(ev[:3]),3)}")
            i = i+1
        
        # Stop the motion once complete
        self.setVelocityZero()

    def translate_along_quaternion(start_pos, quat, local_offset):
        """
        Translate a point in 3D space along the direction defined by a quaternion.

        Parameters
        ----------
        start_pos : array-like, shape (3,)
            The starting point [x, y, z].
        quat : array-like, shape (4,)
            Quaternion [x, y, z, w] representing the orientation.
        local_offset : [x, y, z]
            Distance and direction to move along the quaternion. 
            Note that z is along the quaternion

        Returns
        -------
        end_pos : ndarray, shape (3,)
            The new point after translation.
        """
        # Convert quaternion to rotation object
        rot = R.from_quat(quat)

        # Rotate the local forward vector into world frame
        direction = rot.apply(local_offset)

        # Compute new position
        end_pos = np.array(start_pos) + direction

        return end_pos

#Costants Definitions
class ProbeSurfaceService:
    def __init__(self):
        self.fz_av = 0  # default value for force in z with just gravity
        self.calc_av_force = False # Flag for if the average force in z be calculated
        self.count = 0
        self.sum = 0
        self.zHeight = 0
        self.velocity_controller = ArmVelocity()
        self.distance = 0.01 # Move 10 cm in the probing direction
        self.normalQuat = np.array([0, 0, 0, 1]) # Default to no rotation

    def atiCallback(self, data):  
        size = 100
        threashold = 0.05

        if(self.calc_av_force): #For Calculating the Average Force
            self.sum+=data.fz
            self.count = self.count + 1
            # print(sum)
            if self.count == (size-1):
                self.fz_av = self.sum/size
                self.calc_av_force = False #stop loop from continuing
                self.count = 0
                self.sum = 0
                # print(fz_av)
        if(self.velocity_controller.probing):
            if data.fz > (self.fz_av + threashold):
                self.count = self.count + 1
                if self.count > 20:
                    print(f"Force at zHeight: {round(data.fz,5)}")
                    self.velocity_controller.probing = False
                    self.count = 0

        #Interested in Fz
        # print(data.fx)
        # average_force = 0;

    def endpointCallback(self, data):
        pos = data.pose.position
        self.position = [pos.x, pos.y, pos.z]

    def handle_probe_surface(self, req):
        print("Request Recieved")
        self.count = 0
        self.sum = 0
        self.fz_av = 0
        self.calc_av_force = False #Trigger an Average Force Calculation
        htm = self.velocity_controller.getPos()
        self.start_position = [htm[0][3], htm[1][3], htm[2][3]]
        self.velocity_controller.probing = False
        print(f" Start position: {self.start_position}")
        self.normalQuat = req.normalQuat

        r = rospy.Rate(2)

        self.calc_av_force = True
        while(self.fz_av == 0): #Would Be Better To Do This With An Action Server
            r.sleep()  
        print(f"Average Force: {self.fz_av}")

        self.velocity_controller.probing = True #Start Probing Action

        estimated_end_position = ArmVelocity.translate_along_quaternion(self.start_position, self.normalQuat, [1, 0, 0])

        self.velocity_controller.optimToPoint(estimated_end_position, self.normalQuat, 0.001) # Move
        self.velocity_controller.setVelocityZero()

        htm = self.velocity_controller.getPos()
        self.end_position = [htm[0][3], htm[1][3], htm[2][3]]
        
        # Calculate the distance between start and end position
        self.distance = np.linalg.norm(np.array(self.end_position[0:3]) - np.array(self.start_position[0:3]))
        print(f"zDistance from start position to surface: {round(self.distance,5)}")
        return(self.distance)


    def probeSurfaceServer(self):
        self.count = 0
        self.sum = 0
        self.fz_av = 0
        self.calc_av_force = False #Trigger an Average Force Calculation
        self.probing = False

        print("Initializing node... ")
        s = rospy.Service('probe_surface', Probe, self.handle_probe_surface)

        move = ArmVelocity()
        exiting = move.clean_shutdown
        
        rospy.on_shutdown(exiting)
        r = rospy.Rate(1)
        print("In Function after")

        #initialise Other Subscriber and Publishers
        rospy.Subscriber("/atiForces", AtiMsg, self.atiCallback)
        rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.endpointCallback)

        
        if(exiting != True):
            print("Wating For Probe Command")
            rospy.spin()



if __name__ == '__main__':
    rospy.init_node("probe_surface_server")
    server = ProbeSurfaceService()
    server.probeSurfaceServer()
