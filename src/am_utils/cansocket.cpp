/*
 * cansocket.cpp
 *
 *  Created on: Feb. 20, 2018
 *      Author: AutoModality
 */

//#include "cansocket.h"
#include <am_utils/cansocket.h>

//
// Constructors
//

CanSocket::CanSocket(std::string ifname, int can_id) :
	ifname_(ifname), can_id_(can_id), can_key_(1)
{
	init(ifname);
}

//
// Destructor
//

CanSocket::~CanSocket()
{
	close(s_);
}

int CanSocket::init(std::string ifname)
{
	struct sockaddr_can addr;
	struct ifreq ifr;

	// Open can socket
	if((s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
	{
		perror("Error while opening socket");
		return -1;
	}

	//To meet multi user needs the local loopback is enabled by default.
	//Till now BlinkM LED and LW20 altimeter will not talk, so disable it.
	//It seems "relieve" the stuck problem (stuck at CAN read); it doesn't solve the stuck problem, but much better.
	// The following two lines are for xWR1443
	int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
    setsockopt(s_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));
	// The following two lines are for xWR1642
	//const int canfd_on = 1;
    //setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	struct timeval tv;
	tv.tv_sec = 1;        // 1 Secs Timeout
	tv.tv_usec = 0;        // Not init'ing this can cause strange errors
	setsockopt(s_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

	addr.can_family  = AF_CAN;

	//ifr.ifr_name = ifname.c_str(); incompatible types in assignment of ‘const char*’ to ‘char [16]’
	strcpy(ifr.ifr_name, ifname.c_str());
	//ROS_INFO_STREAM("Interface name: " << ifr.ifr_name);
	if(ioctl(s_, SIOCGIFINDEX, &ifr) < 0)
	{
		perror("SIOCGIFINDEX");
		return -1;
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	std::cout << ifr.ifr_name << " at index " << ifr.ifr_ifindex << std::endl;
	
	if(bind(s_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("Error in socket bind");
		return -2;
	}
	
	return 0;
}

//
// Methods
//

int CanSocket::writeCAN(struct can_frame &frame)
{
	int nbytes;
	if (getCANKey())
	{
		nbytes = write(s_, &frame, sizeof(struct can_frame));
		releaseCANKey();
	}
	else
	{
		std::cout << "Failed to get access the CAN bus for write." << std::endl;

		return -1;
	}

	std::cout << "Wrote " <<  nbytes << " to CAN bus." << std::endl;
	std::cout << "frame.can_id: " <<  frame.can_id << std::endl;
	std::cout << "frame.can_dlc: " <<  (int)frame.can_dlc << std::endl;
	for (int i = 0; i < frame.can_dlc; i++)
	{
		std::cout << "frame.data[" << i << "]: " <<  (int)frame.data[i] << std::endl;
	}
	return 0;
}

int CanSocket::readCAN(struct can_frame *frame)
{
	int nbytes;

	if (getCANKey())
	{
		nbytes = read(s_, frame, sizeof(struct can_frame));
		
		// paranoid check ...
		if (nbytes < (int)sizeof(struct can_frame))
		{
			printf("nbytes: %d.\n", nbytes);
			printf("sizeof(struct can_frame): %d. \n", (int)sizeof(struct can_frame));
			printf("Read CAN: incomplete CAN frame. \n");
			
			releaseCANKey();
			
			return -1;
		}
		
		releaseCANKey();
	}
	else
	{
		printf("Failed to get access the CAN bus for read. \n");

		return -1;
	}

	std::cout << "Read " <<  nbytes << " from CAN bus." << std::endl;
	std::cout << "frame->can_id: " <<  frame->can_id << std::endl;
	std::cout << "frame->can_dlc: " <<  (int)frame->can_dlc << std::endl;
	for (int i = 0; i < frame->can_dlc; i++)
	{
		std::cout << "frame->data[" << i << "]: " <<  (int)frame->data[i] << std::endl;
	}

	return nbytes;
}

int CanSocket::writeCAN(struct canfd_frame &frame)
{
	int nbytes;
	if (getCANKey())
	{
		nbytes = write(s_, &frame, sizeof(struct canfd_frame));
		releaseCANKey();
	}
	else
	{
		printf("Failed to get access the CAN bus for write.\n");

		return -1;
	}

	std::cout << "Wrote " <<  nbytes << " to CAN bus." << std::endl;
	std::cout << "frame.can_id: " <<  frame.can_id << std::endl;
	std::cout << "frame.len: " <<  (int)frame.len << std::endl;
	std::cout << "frame.flags: " <<  (int)frame.flags << std::endl;
	for (int i = 0; i < frame.len; i++)
	{
		std::cout <<  "frame.data[" << i << "]: " <<  (int)frame.data[i] << std::endl;
	}

	return 0;
}

int CanSocket::readCAN(struct canfd_frame *frame)
{
	int nbytes;

	if (getCANKey())
	{
		nbytes = read(s_, frame, sizeof(struct canfd_frame));
		
		if (nbytes < 0)
		{
			perror("can fd raw socket read");
			return 1;
		}

		//ROS_DEBUG_STREAM("sizeof(struct canfd_frame): " << (int)sizeof(struct canfd_frame));
		//ROS_DEBUG_STREAM("nbytes: " << nbytes);
		//ROS_DEBUG_STREAM("nbytes < sizeof(struct canfd_frame): " << (nbytes < (int)sizeof(struct canfd_frame)));
		// paranoid check ...
		if (nbytes < (int)sizeof(struct canfd_frame))
		{
			printf("Read CAN: incomplete CAN frame.\n");
			return -1;
		}

		releaseCANKey();
	}
	else
	{
		printf("Failed to get access the CAN bus for read.\n");

		return -1;
	}

	std::cout << "Read " <<  nbytes << " from CAN bus." << std::endl;
	std::cout << "frame->can_id: 0x" << std::hex << frame->can_id << std::endl;
	std::cout << "frame->len: " <<  (int)frame->len << std::endl;
	std::cout << "frame->flags: 0x" << std::hex << (int)frame->flags << std::endl;
	for (int i = 0; i < frame->len; i++)
	{
		std::cout << "frame->data[" << i << "]: 0x" << std::hex << (int)frame->data[i] << std::endl;
	}

	return nbytes;
}

bool CanSocket::getCANKey()
{
	int i  = 0;

	while (i < CAN_TIMEOUT)
	{
		if(can_key_)
		{
			can_key_ = 0;
			return true;
		}
		i++;
	}

	return false;
}

void CanSocket::releaseCANKey()
{
	can_key_ = 1;
}
