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
#include "MCP2210-Library/ppalm_driver.h"
#include <sstream>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
#include <wam_srvs/LASERControl.h>
#include <wam_srvs/LEDControl.h>

class ppalm_ros_wrapper{

public:
	void init();

private:
	bool led_control(wam_srvs::LEDControl::Request &, wam_srvs::LEDControl::Response &);
	bool laser_control(wam_srvs::LASERControl::Request &, wam_srvs::LASERControl::Response &);

	ros::NodeHandle nh;

	ros::ServiceServer led_control_srv, laser_control_srv;
	ros::Publisher ir_pub;

	sensor_msgs::Range ir;

	//Interface to access the palm drivers
	palm::perception_palm ppalm;
};
