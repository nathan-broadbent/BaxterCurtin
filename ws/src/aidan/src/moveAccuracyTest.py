#!/usr/bin/env python3

import rospy
import pickAndPlace
import searchService
import moveService
import time
import probeSurfaceService

if __name__ == '__main__':
    for i in range(5):
        pickAndPlace.moveTo([0.623, -0.818, 0.22, -0.478, -0.478, -0.521, -0.521]) #Position 1
        _ = input("Proceed to Next Phase")
        pickAndPlace.moveTo([0.623, -0.718, 0.22, -0.478, -0.478, -0.521, -0.521]) #Position 2
        _ = input("Proceed to Next Phase")
        pickAndPlace.moveTo([0.723, -0.818, 0.22, -0.478, -0.478, -0.521, -0.521]) #Position 3
        _ = input("Proceed to Next Phase")
        pickAndPlace.moveTo([0.623, -0.918, 0.22, -0.478, -0.478, -0.521, -0.521]) #Position 4
        _ = input("Proceed to Next Phase")
    # pickAndPlace.moveTo([0.623, -0.818, 0.22, -0.478, -0.478, -0.521, -0.521]) #Position 1
    # _ = input("Proceed to Next Phase")
    