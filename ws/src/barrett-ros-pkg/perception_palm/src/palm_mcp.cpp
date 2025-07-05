/*
	Copyright 2014, 2015 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <palm_mcp.h>
#include <sstream>

const string LED_CONTROL_SRV = "palm/set_led_on";
const string LASER_CONTROL_SRV = "palm/set_laser_on";
const string IR_TOPIC = "palm/ir/range";
const string FIXED_FRAME_TOPIC = "fixed_frame";

bool ppalm_ros_wrapper::led_control(wam_srvs::LEDControl::Request &req, wam_srvs::LEDControl::Response &res){

	if (!req.turn_on.data)
		ppalm.led_off();
	else
		ppalm.led_on();
	return true;
}

bool ppalm_ros_wrapper::laser_control(wam_srvs::LASERControl::Request &req, wam_srvs::LASERControl::Response &res){

	if (!req.turn_on.data)
		ppalm.laser_off();
	else
		ppalm.laser_on();
	return true;
}

void ppalm_ros_wrapper::init(){

	//Initialize the services
	led_control_srv = nh.advertiseService(LED_CONTROL_SRV, &ppalm_ros_wrapper::led_control, this);
    laser_control_srv = nh.advertiseService(LASER_CONTROL_SRV, &ppalm_ros_wrapper::laser_control, this);

	//Initialize the publisher
	ir_pub = nh.advertise<sensor_msgs::Range>(IR_TOPIC, 1000);

	//Set the rate at 1Hz
	ros::Rate loop_rate(200);

	//Set the default parameters for the IR range message
	ir.radiation_type = sensor_msgs::Range::INFRARED;
	ir.header.frame_id = FIXED_FRAME_TOPIC;
	ir.field_of_view = palm::FIELD_OF_VIEW;
	ir.min_range = float(palm::MIN_DIST)/1000;
	ir.max_range = float(palm::MAX_DIST)/1000;

	
	while(ros::ok()){
		ir.header.stamp = ros::Time::now();

		//Convert the IR information in meters
		ir.range = float(ppalm.ir_range())/1000; 

		ir_pub.publish(ir);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_palm_node");
  ppalm_ros_wrapper prw;
  prw.init();
  return 0;
}
