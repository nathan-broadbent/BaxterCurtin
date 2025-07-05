#!/usr/bin/env python3

import rospy
from ati_sensor_ros_driver.rpi_ati_net_ft import NET_FT
import argparse
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState

class ATIDriver(object):
    def __init__(self, host, ref_topic) -> None:
        super().__init__()

        self.host = host
        self.ati_obj = NET_FT(host)
        self.ati_obj.set_tare_from_ft()
        print(self.ati_obj.read_ft_http())
        print(self.ati_obj.try_read_ft_http())

        # Subscriber
        self.ref_time = rospy.Subscriber(ref_topic,JointState,self.get_time)

        # service server
        self.tare_srv = rospy.Service('set_ati_tare',Trigger,self.set_tare)
        self.clr_tare_srv = rospy.Service('clear_ati_tare',Trigger,self.clear_tare)

        # publisher
        self.ft_pub = rospy.Publisher("ati_ft",WrenchStamped,queue_size=1)

        self.ati_obj.start_streaming()

        # loop timer
        self.rate = 1e-1
        pub_timer = rospy.Timer(rospy.Duration(self.rate),self.get_ft)

        
        self.time = rospy.Time.now()
    
    def get_ft(self, event):

        res, ft, status = self.ati_obj.try_read_ft_streaming(0)
        fstamp = rospy.Time.now()
        try:
            w = WrenchStamped()
            w.header.stamp = self.time
            w.header.frame_id = 'right_hand'
            # frame changed from 'ati_frame'
            w.wrench.torque.x = ft[0]
            w.wrench.torque.y = ft[1]
            w.wrench.torque.z = ft[2]
            w.wrench.force.x = ft[3]
            w.wrench.force.y = ft[4]
            w.wrench.force.z = ft[5]

            self.ft_pub.publish(w)
        except :
            self.ft_pub.publish[0,0,0,0,0,0]
            print("Message Error")

    def set_tare(self, req):
        
        self.ati_obj.set_tare_from_ft()
        return TriggerResponse(
            success=True,
            message="ATISensor: The tare is set."
        )
    
    def clear_tare(self, req):

        self.ati_obj.clear_tare()
        return TriggerResponse(
            success=True,
            message="ATISensor: The tare is cleared."
        )
    
    def get_time(self, msg):
        #TLH: pulling time from baxter to allow TF to work
        self.time = msg.header.stamp

def main():
    parser = argparse.ArgumentParser(description="ATI force torque sensor driver service for Robot Raconteur")
    parser.add_argument("--sensor-ip", type=str, default="134.7.44.202", help="the ip address of the ati sensor")
    # IP address updated
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")
    parser.add_argument("--ref-topic", type=str, default="/robot/joint_states", help="the stamped topic that will be used as a reference time for the ROS network (published by Baxter)")

    args,_ = parser.parse_known_args()
    
    rospy.init_node('ati_driver')
    ati_driver_obj = ATIDriver(args.sensor_ip, args.ref_topic)

    rospy.spin()

if __name__ == "__main__":
    main()
