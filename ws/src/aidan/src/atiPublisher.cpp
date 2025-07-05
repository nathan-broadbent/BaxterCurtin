//Stuff for same exiting
#include <signal.h>
#include <stdbool.h>

//ROS STUFF
#include "ros/ros.h"
#include "aidan/AtiMsg.h"

#include <sstream>

//Stuff For Reading The ATI Sensor
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//ROS VARIABLES
#define SAMPLE_FREQUENCY 500

//ATI SENSOR Variables
#define PORT 49152 /* Port the Net F/T always uses */
#define STOP_COMMAND 0 //Command to stop the streaming
#define START_COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 0 /* Will Continue to send data until stopped */
#define IP_ADDRESS "134.7.44.202"
#define SENSOR_SCALING_FACTOR 1000000
#define CUSTOM_SCALING_FACTOR 100

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;
typedef struct response_struct {
	uint32 rdt_sequence;
	uint32 ft_sequence;
	uint32 status;
	int32 FTData[6];
} RESPONSE;

void sigint_handler(int s){
    int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
	int i, a;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */
	// char * AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */

    printf("\n CTRL-C Detected: Closing Gracefully\n");

    /* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1) {
		exit(1);
	}
	
	*(uint16*)&request[0] = htons(0x1234); /* standard header. */
	*(uint16*)&request[2] = htons(STOP_COMMAND); /* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
	
	/* Sending the request. */
	he = gethostbyname(IP_ADDRESS);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}
	send( socketHandle, request, 8, 0 ); //Sending the Packet to Start the Communications
    sleep(0.1);
    exit(s);
}

int main(int argc, char **argv)
{
    //Allowing Gracefull Closing
    signal(SIGINT, sigint_handler);

    //Initiates the Ros Node
    ros::init(argc, argv, "atiForces");
	// ros::sleep(0.1)

    //How to interact with the node
    ros::NodeHandle n;

    //Tells ros the name of the topic to publish on.
    ros::Publisher ati_pub = n.advertise<aidan::AtiMsg>("atiForces", 1000);
    ros::Rate loop_rate(SAMPLE_FREQUENCY);
    uint count = 0;

    int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;			/* Host entry for Net F/T. */
	byte request[8];			/* The request data sent to the Net F/T. */
	RESPONSE resp;				/* The structured response received from the Net F/T. */
	byte response[36];			/* The raw response data received from the Net F/T. */
	int i, a;						/* Generic loop/array index. */
	int err;					/* Error status of operations. */

    /* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1) {
		exit(1);
	}
	
	*(uint16*)&request[0] = htons(0x1234); /* standard header. */
	*(uint16*)&request[2] = htons(START_COMMAND); /* per table 9.1 in Net F/T user manual. */
	*(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
	
	/* Sending the request. */
	he = gethostbyname(IP_ADDRESS);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}
	send( socketHandle, request, 8, 0 ); //Sending the Packet to Start the Communications


    while(ros::ok())
    {
        std_msgs::Header header;
        aidan::AtiMsg msg;

        if(NUM_SAMPLES == 0 | NUM_SAMPLES > count)
        {
            header.seq = count;
            header.stamp = ros::Time::now();
            msg.header = header;

            printf( "\nLoop Number:%d \n", count);
            recv( socketHandle, response, 36, 0 );
            resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
            resp.ft_sequence = ntohl(*(uint32*)&response[4]);
            resp.status = ntohl(*(uint32*)&response[8]);
			// printf("Waiting Here\n");
            for( i = 0; i < 6; i++ ) {
                resp.FTData[i] = ntohl(*(int32*)&response[12 + i * 4]);
            }
            msg.fx = (float)resp.FTData[0]/SENSOR_SCALING_FACTOR/CUSTOM_SCALING_FACTOR;
            msg.fy = (float)resp.FTData[1]/SENSOR_SCALING_FACTOR/CUSTOM_SCALING_FACTOR;
            msg.fz = (float)resp.FTData[2]/SENSOR_SCALING_FACTOR/CUSTOM_SCALING_FACTOR;
            msg.tx = (float)resp.FTData[3]/SENSOR_SCALING_FACTOR/CUSTOM_SCALING_FACTOR;
            msg.ty = (float)resp.FTData[4]/SENSOR_SCALING_FACTOR/CUSTOM_SCALING_FACTOR;
            msg.tz = (float)resp.FTData[5]/SENSOR_SCALING_FACTOR/CUSTOM_SCALING_FACTOR;
			// printf("data %f",msg.fx);
            ati_pub.publish(msg);
            count++;
			// printf("Got Here\n");
        }
		// printf("Got Here 2 \n");
        loop_rate.sleep();
		// printf("Got Here 3 \n");
        // ROS_INFO("%s", msg.data.c_str());
    }
    //Stopping the ATI Sensor From Streaming Data in event of roscore closure
    *(uint16*)&request[2] = htons(STOP_COMMAND); /* per table 9.1 in Net F/T user manual. */
    send( socketHandle, request, 8, 0 ); //Sending the Packet to Stop the Communications
    return 0;
}


