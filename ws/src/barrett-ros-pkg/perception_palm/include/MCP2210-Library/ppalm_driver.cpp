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
#include "ppalm_driver.h"
#include <iomanip>
using namespace palm;
/*Configures the pin as input and output
 *
 */
perception_palm::perception_palm(){

	// Initializing the MCP device
	handle = InitMCP2210();
	if (handle ==0 ) {
	    printf("Error Initializing the device. Switch to sudo or connect the USB to the correct port");
	    exit(1);
	}

	chipDef = GetChipSettings(handle);

	// Configure the LED pin as a General Purpose output
	chipDef.GP[5].PinDesignation = 0x00;
	chipDef.GP[5].GPIODirection = GPIO_DIRECTION_OUTPUT;

	// Configure the Laser pin as a General Purpose output
	chipDef.GP[4].PinDesignation = 0x00;
	chipDef.GP[4].GPIODirection = GPIO_DIRECTION_OUTPUT;

	// Turn on the IR range sensor
        chipDef.GP[2].PinDesignation = 0x00;
        chipDef.GP[2].GPIODirection = GPIO_DIRECTION_OUTPUT;
        chipDef.GP[2].GPIOOutput = 0;

	// Set GP1 as chip select
	chipDef.GP[1].PinDesignation = GP_PIN_DESIGNATION_CS;
	chipDef.GP[1].GPIODirection = GPIO_DIRECTION_OUTPUT;
	chipDef.GP[1].GPIOOutput = 1;

	// Apply the settings
	int r = SetChipSettings(handle, chipDef);

	// Configure SPI and the CS
	SPITransferSettingsDef def;
	def = GetSPITransferSettings(handle);
	
	def.SPIMode = 0;
	def.ActiveChipSelectValue = 0x1fd;
	def.IdleChipSelectValue = 0x1fb;
	def.BitRate = 50000l;
	def.BytesPerSPITransfer =2;

    	r = SetSPITransferSettings(handle, def);
	
	// Return an error if the parameters are not successfully set
	if (r != 0) {
	    printf("Error setting SPI parameters.\n");
	    exit(1);
	}

	MAX_VOLT = 2750;
	MID_VOLT = 1700;
	MIN_VOLT = 550;
	POSITIVE_ERROR = 0;
	NEGATIVE_ERROR = 0;
	perception_palm::readFromfile();
	//std::cout<<"The updated values are "<<MAX_VOLT<<" "<<MIN_VOLT<<" "<<POSITIVE_ERROR<<" "<<NEGATIVE_ERROR<<std::endl;
	// Calculate the Resolution for the IR sensor
        resolution = ((float(std::abs(MAX_DIST - MID_DIST))/float(std::abs(MAX_VOLT - MID_VOLT))) + (float(std::abs(MIN_DIST - MID_DIST))/float(std::abs(MIN_VOLT - MID_VOLT))))/2;

}

/* Queries for IR range information
 * @returns the distance in mm
 */
double perception_palm::ir_range(){
	byte spiCmdBuffer[2];
	spiCmdBuffer[0]=0x06;
	spiCmdBuffer[1]=0x00;
	SPIDataTransferStatusDef def1 = SPISendReceive(handle, spiCmdBuffer, 0, 2);
	unsigned int binary_volt = ((def1.DataReceived[0] <<8) | def1.DataReceived[1]);
	unsigned int final_volt = ((binary_volt >>1) & 0x0fff);
	
	//Debug
	//printf("The invdl bits are %d %d and %d %d, Binary Volt is %d and final volt is %d\n",def1.DataReceived[0], spiCmdBuffer[0], def1.DataReceived[1], spiCmdBuffer[1], binary_volt, final_volt);
	//printf("The final volt is %d\n", final_volt);
	if (final_volt > MAX_VOLT || final_volt < MIN_VOLT)
		return 0;
	float dist = (float(resolution) * (MAX_VOLT - final_volt)) + MIN_DIST + POSITIVE_ERROR - NEGATIVE_ERROR ;

	return ((dist <= MAX_DIST && dist >= MIN_DIST) ? dist:0);
}

/* Turn on the led
 * @returns error code
 */
int perception_palm::led_on(){

	chipDef.GP[5].GPIOOutput = 1;
	int err = SetChipSettings(handle, chipDef);
	return err;
}

/* Turn off the led
 * @returns error code
 */
int perception_palm::led_off(){
	chipDef.GP[5].GPIOOutput = 0;
	int err = SetChipSettings(handle, chipDef);
	return err;
}

/* Turn on the laser
 * @returns error code
 */
int perception_palm::laser_on(){

	chipDef.GP[4].GPIOOutput = 0;
	int err = SetChipSettings(handle, chipDef);
	return err;
}

/* Turn off the laser
 * @returns error code
 */
int perception_palm::laser_off(){
	chipDef.GP[4].GPIOOutput = 1;
	int err = SetChipSettings(handle, chipDef);
	return err;
}

/* Release the hand
 * 
 */
perception_palm::~perception_palm(){
    	ReleaseMCP2210(handle);
}


void perception_palm::readFromfile(){
	string line;
	unsigned int a[5];
  	ifstream myfile(palm::file_name);
  	if (myfile.is_open())
  	{
		int i=0;
    	while ( getline (myfile,line) )
    	{
		a[i]=atoi(line.c_str());
		i++;
    	}
    	myfile.close();
  	}
  	else 
		std::cerr << "Unable to open file "<<palm::file_name<<std::endl;;
	
	if(a[0]!=0)
		MAX_VOLT = a[0];
	if(a[1]!=0)
		MID_VOLT = a[1];
	if(a[2]!=0)
		MIN_VOLT = a[2];
	if(a[3]!=0)
		POSITIVE_ERROR = a[3];
	if(a[4]!=0)
		NEGATIVE_ERROR = a[4];

	//std::cout<<"The values are "<<MAX_VOLT<<" "<<MIN_VOLT<<" "<<POSITIVE_ERROR<<" "<<NEGATIVE_ERROR<<std::endl;
  	return;
}

int main(int argc, char** argv){
return 0;
}
