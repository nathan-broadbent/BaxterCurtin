/**
 *    Copyright 2012, Kerry D. Wong
 * 
 *      http://www.kerrywong.com
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "mcp2210.h"
#include <iostream>
#include<fstream>
#include <boost/thread.hpp>

using namespace std;

unsigned int binary_data=0, MAX_VOLT = 0, MIN_VOLT = 0, MID_VOLT =0, MAX_VOLT_HIGH = 2800, MAX_VOLT_LOW = 2600, MIN_VOLT_HIGH = 700, MIN_VOLT_LOW = 400, POS_ERR = 0, NEG_ERR = 0, MID_VOLT_LOW = 1500, MID_VOLT_HIGH = 1900;
const char * file_name="ir_calibration.txt";


void TestMCP3204(hid_device* handle) {
	while(true){
	    ChipSettingsDef chipDef;

	    //set GPIO pins to be CS
	    chipDef = GetChipSettings(handle);

		chipDef.GP[1].PinDesignation = GP_PIN_DESIGNATION_CS;
		chipDef.GP[1].GPIODirection = GPIO_DIRECTION_OUTPUT;
		chipDef.GP[1].GPIOOutput = 1;

		chipDef.GP[2].PinDesignation = 0x00;
		chipDef.GP[2].GPIODirection = GPIO_DIRECTION_OUTPUT;
		chipDef.GP[2].GPIOOutput = 0;

	    int r = SetChipSettings(handle, chipDef);

	    //configure SPI
	    SPITransferSettingsDef def;
	    def = GetSPITransferSettings(handle);

	    //chip select is GP1
	    def.SPIMode = 0;
	    def.ActiveChipSelectValue = 0x1fd;
	    def.IdleChipSelectValue = 0x1fb;
	    def.BitRate = 50000l;
	    def.BytesPerSPITransfer =2;

	    r = SetSPITransferSettings(handle, def);

	    if (r != 0) {
		printf("Error setting SPI parameters.\n");
		return;
	    }
	    
	    byte spiCmdBuffer[2];
	    spiCmdBuffer[0]=0x06;
	    spiCmdBuffer[1]=0x00;
	    SPIDataTransferStatusDef def1 = SPISendReceive(handle, spiCmdBuffer, 0, 2); 
	    unsigned int merged_bytes = ((def1.DataReceived[0] <<8) | def1.DataReceived[1]);
	    binary_data = ((merged_bytes >>1) & 0x0fff);
	    
	    //Debug
	    //printf("The invdl bits are %d and %d, Binary Volt is %d\n",def1.DataReceived[0], def1.DataReceived[1], binary_data);
	}
}

bool capture_data(unsigned int &output,unsigned int min_range,unsigned int max_range){
char ch;
	while(true){
	while(cin.get() != '\n'){
	std::cout<<"Hit Enter once ready"<<std::endl;
	}
	if (binary_data > min_range && binary_data < max_range){
		output = binary_data;
		return true;
	}
	else
		std::cout<<"Sorry! Could you please check the distance of the obstacle and try it again?"<<std::endl;
	}
	return false;
}

void writeTofile(){
ofstream myfile(file_name);
  if (myfile.is_open())
  {
    myfile << MAX_VOLT <<"\n";
    myfile << MID_VOLT <<"\n";
    myfile << MIN_VOLT <<"\n";
    myfile << POS_ERR <<"\n";
    myfile << NEG_ERR <<"\n";
    myfile.close();
  }
  else 
	std::cerr << "Unable to open file"<<file_name;
  return ;
}

int validInput()
{
    int x;
    std::cin >> x;
    while(std::cin.fail())
    {
        std::cin.clear();
        //std::cin.ignore(std:numeric_limits<std::streamsize>::max(),'\n');
	std::cin.ignore(50,'\n');
        std::cout << "Bad entry.  Enter a NUMBER in the range (0-50): "<<std::endl;;
        std::cin >> x;
    }
    return x;
}

int main(int argc, char** argv) {
    hid_device *handle;

    /**
     * initializing the MCP2210 device.
     */
    handle = InitMCP2210();

    if (handle == 0) {
        printf("ERROR opening device. Try using sudo.\n");
        exit(-1);
    }

	boost::thread t(&TestMCP3204,handle);
	
	std::cout<<"Place an obstacle at a distance of 30 mm from the IR range finder and hit enter"<<std::endl;
	bool res1 = capture_data(MAX_VOLT, MAX_VOLT_LOW, MAX_VOLT_HIGH);

	std::cout<<"Great! Place an obstacle at a distance of 250 mm from the IR range finder and hit enter"<<endl;
	bool res2 = capture_data(MID_VOLT, MID_VOLT_LOW, MID_VOLT_HIGH);
	
	std::cout<<"Good! Place an obstacle at a distance of 500 mm from the IR range finder and hit enter"<<endl;
	bool res3 = capture_data(MIN_VOLT, MIN_VOLT_LOW, MIN_VOLT_HIGH);
	
	std::cout<<"Wonderful! Enter a Positive error offset (in mm). Typically, this would be between 0 - 50 mm. Enter 0 if you are unsure"<<std::endl;
	/*int ctr=0;
	do{
		POS_ERR = validInput();
		if (ctr!=0)
			std::cout << "Bad entry.  Enter a NUMBER in the range (0-50): ";
		ctr++;
	}
	while(POS_ERR>50 || POS_ERR<0);

	std::cout<<"Ok, now enter a Negative error offset (in mm). Typically, this would be between 0 - 50 mm. Enter 0 if you are unsure"<<std::endl;
	ctr=0;
	do{
		NEG_ERR = validInput();
		if (ctr!=0)
			std::cout << "Bad entry.  Enter a NUMBER in the range (0-50): ";
		ctr++;
	}
	while(POS_ERR>50 || POS_ERR<0);*/

	if (res1 && res2 && res3)
		writeTofile();
	else
		std::cout<<"Something went wrong. Please check the connections and run the program again"<<std::endl;
	std::cout<<"Thank you. The calibration is done successfully"<<std::endl;

    /**
     * release the handle
     */
    ReleaseMCP2210(handle);

    return 0;
}
