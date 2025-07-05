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
#ifndef PPALM_DRIVER_H
#define	PPALM_DRIVER_H

#include <cstdlib>
#include <fstream>
#include "mcp2210.h"
#include <iostream>
using namespace std;

namespace palm{

// Initialize the constants for the IR calibration. Dist in mm

const int MAX_DIST = 500;
const int MID_DIST = 250;
const int MIN_DIST = 25;
const int FIELD_OF_VIEW = 6;
const char* file_name="ir_calibration.txt";
class perception_palm{

private:
	hid_device *handle;
	ChipSettingsDef chipDef;
	float resolution;
	void readFromfile();
	int MAX_VOLT, MIN_VOLT, MID_VOLT, POSITIVE_ERROR, NEGATIVE_ERROR;

public:
	perception_palm();
	int led_on();
	int led_off();
	int laser_on();
	int laser_off();
	double ir_range();
	~perception_palm();
};
}// namespace
#endif
