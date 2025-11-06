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
        """
        Moves the end effector to a specified position using optimization to calculate joint velocities
        Will stop if self.probing is set to False
        will move in a straight line to the target position
        will maintain current orientation unless a rotation is provided
        pos: Target position as a list or array [x, y, z]
        rot: Target rotation as a list or array quaternion [qx, qy, qz, qw]
        lin_vmax: Maximum linear velocity for the end effector
        """
        # Force control parameters
        fx_setpoint = 5.0      # Desired force along the normal (N)
        deadband_N = 0.5       # Deadband around the setpoint (N)
        Kf = 0.001             # Force control gain (m/s per N of error)
        max_force_vel = 0.01   # Maximum velocity correction due to force control

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
            
            # --- Velocity control component ---
            ev, arrived_ = rtb.p_servo(Te, Tep, gain=gain, threshold=0.005, method='angle-axis')
            vel = np.linalg.norm(ev[:3])

            if (vel > lin_vmax):
                ev[:3] = np.dot(lin_vmax/vel, ev[:3])

            # --- Force control component ---
            # construct force vector in the sensor frame
            force_sensor = np.array([self.fx, getattr(self, 'fy_filtered', 0.0), getattr(self, 'fz_filtered', 0.0)])

            # Get end-effector rotation (world/base frame)
            # Te is the 4x4 transform returned by getJacobianAndPos() call earlier in loop
            R_ee = Te[:3, :3]   # rotation from EE frame -> world frame

            # form force in world frame depending on where ATI reports forces
            force_world = R_ee @ force_sensor

            # end-effector x-axis in world coords (unit)
            normal_world = R_ee[:, 0]
            norm_n = np.linalg.norm(normal_world)
            if norm_n < 1e-6:
                normal_world = np.array([1.0, 0.0, 0.0])
            else:
                normal_world = normal_world / norm_n

            # project force onto normal
            f_along_normal = float(np.dot(force_world, normal_world))

            # error and deadband
            f_error = fx_setpoint - f_along_normal
            if abs(f_error) < deadband_N:
                f_error = 0.0

            # compute velocity correction along the normal (in world frame)
            corr_vel = Kf * f_error * normal_world

            # clamp correction magnitude
            corr_mag = np.linalg.norm(corr_vel)
            if corr_mag > max_force_vel:
                corr_vel = (max_force_vel / corr_mag) * corr_vel

            # apply the correction to linear part of ev (vector in world frame)
            ev[:3] = ev[:3] + corr_vel

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

class SearchService:
    def __init__(self):
        self.count = 0
        self.searching = False
        self.fy = 0
        self.firstExecution = True
        # TODO: calibrate this value
        self.searchGap = 0.05 # 5cm Gap for Searching
        self.move = ArmVelocity()

    def atiCallback(self, data):        
        threashold = self.fy + 0.05
        if(self.firstExecution):
            self.move.fx = data.fx
            self.move.fy = data.fy
            self.move.fz = data.fz
            self.fy = data.fy
            self.threshold = self.fy + 0.05
            print(f"Current Fy: {self.fy}")
            self.firstExecution = False
        # Slighlty recalibrate threashold downwards over time
        if(data.fy < threashold - 0.15):
            threashold = threashold - 0.001

        if(self.searching):
            # print(data.fy)
            if data.fy > (threashold):
                count = count + 1
                if count > 10:
                    print(data.fy)
                    print("Greater than threashold")
                    searching = False
                    count = 0


    def handle_search_hole(self, req):
        print("Search For Hole Called!")
        self.firstExecution = True #Calc Current force in the Y Direction
        self.count = 0
        self.searching = False
        self.operatingQuat # The search operation should be done a quaternion slightly offset of the normal

        self.center = req.position
        self.normalQuat = req.NormalQuat
        zDistance = req.zDistance
        
        width = req.searchWidth
        length = req.searchLength

        searching = True  #Start Search Opperation
        repeats = 2
        rep = 0
        
        xCount = width/0.03
        yCount = length/0.03

        print(f"width: {width}")
        print(f"length: {length}")
        while(searching and rep < repeats):
            div = 4
            # TODO: Generalise - We should calculate the number of divisions based on the width and length
            # TODO: Should be using end point on the plane perpendicular to the normal quaternion
            if self.searching and rep < repeats:
                for i in range(xCount):
                    dx = (self.center[0]-width/2 + (width/(xCount-1)*i)) - self.center[0]
                    dy_start = (self.center[1]-length/2) - self.center[1]
                    dy_end = (self.center[1]+length/2) - self.center[1]
                    # Above next start point on surface
                    end_point = transform_offset(self.center, self.normalQuat, [dx, dy_start, 0.005])
                    self.move.optimToPoint(end_point, self.operatingQuat, 0.05) # Move to end point
                    # Move down start point
                    end_point = transform_offset(self.center, self.normalQuat, [dx, dy_start, 0])
                    self.move.optimToPoint(end_point, self.operatingQuat) # Move to end point
                    # Move to end point, dragging along the surface
                    end_point = transform_offset(self.center, self.normalQuat, [dx, dy_end, 0])
                    self.move.optimToPoint(end_point, self.operatingQuat, 0.01)
            rep += 1
            zheight -= 0.005 #! I don't understand why this line is necessary
        success = not searching
        searching = False
        return success

    def searchService(self):
        print("Search Service Starting")
        self.count = 0
        self.probing = False
        self.fy = 0.35
        self.firstExecution = False
        
        print("Initializing node... ")
        rospy.Subscriber("/atiForces", AtiMsg, self.atiCallback)
        s = rospy.Service("search_hole", searchHole, self.handle_search_hole)
        
        exiting = self.move.clean_shutdown
        rospy.on_shutdown(exiting)

        if(exiting != True):
            print("Wating For Probe Command")
            rospy.spin()

def transform_offset(center_pos, quat, local_offset):
    """
    Transforms a local offset (dx, dy, dz) into world coordinates based on a quaternion.

    Parameters
    ----------
    center_pos : array-like (3,)
        The known centre point [x, y, z].
    quat : array-like (4,)
        Quaternion [x, y, z, w] representing surface orientation.
    local_offset : array-like (3,)
        Offset from the centre in the local (surface-aligned) frame.

    Returns
    -------
    world_pos : ndarray (3,)
        Transformed point in world coordinates.
    """
    rot = R.from_quat(quat)
    offset_world = rot.apply(local_offset)
    world_pos = np.array(center_pos) + offset_world
    return world_pos

if __name__ == '__main__':
    rospy.init_node("search_controller")
    service = SearchService()
    service.searchService()