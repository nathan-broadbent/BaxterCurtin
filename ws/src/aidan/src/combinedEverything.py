#!/usr/bin/env python3
import rospy
import pickAndPlace
import searchService
import moveService
import time
import probeSurfaceService

from aidan.srv import (
    probe,
    searchHole
)

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


if __name__ == '__main__':

    insertFirst = True
    insertSecond = False

    

    if insertFirst:
        hole_x_pos = 0.81
        hole_y_pos = -0.14
        angle_offset = 0.01
        zHeight = 0.09363
        searchWidth = 0.05
        searchLength = 0.02

        # pickAndPlace.moveTo([0.5, -0.6, 0.13, -0.478, -0.478, -0.521, -0.521])
        # _ = input("test")

        # Pick Workpiece Operation
        print("Performing Pick and Place")
        pickAndPlace.pickTube()
        R = "a"
        while R=="a":
            # Probe Operation
            print("Moving to Probing Location")
            pickAndPlace.moveTo([0.828,0.0,0.25, -0.478, -0.478, -0.521, -0.521])
            

            print("Performing Probing Action")
            resp = probeSurfaceClient()
            zHeight = resp.zHeight - 0.005
            print(f"zHeight: {zHeight}")
            R = input("Please enter a to repeat")
        _ = input("Enter to continue")

        # Hole Searching Operation
        print("Moving to search position")
        pickAndPlace.moveTo([hole_x_pos,hole_y_pos,zHeight+0.15, -0.478, -0.478, -0.521, -0.521])
        print("Performing Search")
        searchHoleClient(hole_x_pos, hole_y_pos+angle_offset,zHeight,searchWidth, searchLength)

        # _ = input("Enter to continue")
        #Orientation Adjustment Operation
        print("Correcting Orientation for Insertion")
        pickAndPlace.moveToRelative([0.005,-angle_offset,0,-0.478, -0.478, -0.521, -0.521])

        #Final Insert Operation
        _ = input("Please press any key to proceed with final insertion!")
        print("performing Final Insertion")
        pickAndPlace.moveToRelative([0,0,-0.01,-0.478, -0.478, -0.521, -0.521])

        #Remove Hand Operation
        _ = input("Please confirm that the hand has been opened!")
        pickAndPlace.moveToRelative([0,0,0.2,-0.478, -0.478, -0.521, -0.521])
        _ = input("Please confirm that the hand is closed")
        pickAndPlace.moveTo([0.5, -0.7, 0.2, -0.478, -0.478, -0.521, -0.521])

        print("Process Compleate")

    #Insertion of Object 2
    
    if insertSecond:
        hole_x_pos = 0.81
        hole_y_pos = -0.14
        angle_offset = 0.01
        zHeight = 0.09363
        searchWidth = 0.05
        searchLength = 0.02

        # Pick Workpiece Operation
        print("Performing Pick and Place")
        pickAndPlace.pickTubePos(0.6,-0.9,0.04)

        # # Probe Operation
        # print("Moving to Probing Location")
        # pickAndPlace.moveTo([0.828,0.0,0.25, -0.478, -0.478, -0.521, -0.521])
        # print("Performing Probing Action")
        # resp = probeSurfaceClient()
        # zHeight = resp.zHeight - 0.005
        # print(f"zHeight: {zHeight}")


        # # Hole Searching Operation
        # print("Moving to search position")
        # pickAndPlace.moveTo([hole_x_pos,hole_y_pos,zHeight+0.15, -0.478, -0.478, -0.521, -0.521])
        # print("Performing Search")
        # searchHoleClient(hole_x_pos, hole_y_pos+angle_offset,zHeight,searchWidth, searchLength)

        # #Orientation Adjustment Operation
        # print("Correcting Orientation for Insertion")
        # pickAndPlace.moveToRelative([0.005,-angle_offset,0,-0.478, -0.478, -0.521, -0.521])

        # #Final Insert Operation
        # _ = input("Please press any key to proceed with final insertion!")
        # print("performing Final Insertion")
        # pickAndPlace.moveToRelative([0,0,-0.01,-0.478, -0.478, -0.521, -0.521])

        # #Remove Hand Operation
        # _ = input("Please confirm that the hand has been opened!")
        # pickAndPlace.moveToRelative([0,0,0.2,-0.478, -0.478, -0.521, -0.521])
        # _ = input("Please confirm that the hand is closed")
        # pickAndPlace.moveTo([0.5, -0.8, 0.13, -0.478, -0.478, -0.521, -0.521])

        # print("Process Compleate")
    

    #Hole POS: 
    # pickAndPlace.moveTo([0.81,-0.14,0.13, -0.478, -0.478, -0.521, -0.521])
    # pickAndPlace.moveTo([0.8, 0, 0.2, -0.478, -0.478, -0.521, -0.521])