#!/usr/bin/env python3

# import numpy as np
# import matplotlib.pyplot as plt
# import time
# import random
# import rospy
# from aidan.msg import AtiMsg
# import threading

# fx = []
# fy = []
# fz = []
# tx = []
# ty = []
# tz = []

# fig = None
# ax = None
# i = 0

# def callback(data):
#     # rospy.loginfo(rospy.get_caller_id() + "I head %s", data.fx)
#     fx.append(data.fx)
#     fy.append(data.fy)
#     fz.append(data.fz)
#     i = data.header.seq
#     show_graph()


# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber("atiForces", AtiMsg, callback)

#     # spin() simply keeps python from exiting until this node is stopped
    

# def show_graph():
#     ax.plot(fx, color='r')
#     fig.canvas.draw()
#     ax.set_xlim(left=max(0, i - 50), right=i + 3)
   

    


# if __name__ == '__main__':
    
#     fig = plt.figure()
#     ax = fig.add_subplot()
#     fig.show()
#     # t1 = threading.Thread(target=show_graph, args=())
#     # t2 = threading.Thread(target=listener, args=())

#     listener()
#     plt.show(block=True)



 
# # for i in range(n):
# #     x.append(random.randint(0,100))
# #     ax.plot(x, color='r')
# #     fig.canvas.draw()
# #     ax.set_xlim(left=max(0, i - 50), right=i + 3)
# #     time.sleep(0.01)
# # plt.show()

# https://stackoverflow.com/questions/35145555/python-real-time-plotting-ros-data

import matplotlib.pyplot as plt
import rospy
import numpy as np
from matplotlib.animation import FuncAnimation
from aidan.msg import AtiMsg
# y_axis = [i for i in range(2000)]

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], '-r')
        self.x_data, self.y_data = [] , []

    def plot_init(self):
        self.ax.set_xlim(0, 100000)
        self.ax.set_ylim(-15, 4)
        return self.ln
    
    def force_callback(self, msg):
        self.y_data.append(msg.fx)
        x_index = len(self.x_data)
        self.x_data.append(x_index+1)
    
    def update_plot(self, frame):
        self.ln.set_data(self.x_data, self.y_data)
        # if(self.y_data > 8000)
        return self.ln


rospy.init_node('force_grapher')
vis = Visualiser()
sub = rospy.Subscriber('atiForces', AtiMsg, vis.force_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True) 