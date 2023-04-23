/*
 * am_network_utils.cpp
 *
 *  Created on: Jun 2, 2015
 *      Author: dan
 */

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>

#include <rclcpp/rclcpp.hpp>

#include <am_utils/am_network_utils.h>

namespace am {

AMNetworkUtils::AMNetworkUtils() {
	// TODO Auto-generated constructor stub

}

AMNetworkUtils::~AMNetworkUtils() {
	// TODO Auto-generated destructor stub
}

// get sockaddr, IPv4 or IPv6:
void *AMNetworkUtils::get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

void AMNetworkUtils::printSource(struct sockaddr_storage their_addr, socklen_t addr_len, long bytesReceived)
{
    char s[INET6_ADDRSTRLEN];

    const char *addr = inet_ntop(their_addr.ss_family,
                                 get_in_addr((struct sockaddr *)&their_addr),
                                 s, sizeof s);

	std::cout << "got " << bytesReceived << " byte packet from " <<  addr << std::endl;
 }

int AMNetworkUtils::openIncomingSocket(const char *name, const char *port)
{
	std::cout << "opening " << name << " incoming socket on port " << port << " ..." << std::endl;

	struct addrinfo hints;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE; // use my IP

	struct addrinfo *servinfo;
	int rv;
	if ((rv = getaddrinfo(NULL, port, &hints, &servinfo)) != 0) {
		std::cout << "getaddrinfo:"  <<  gai_strerror(rv) << std::endl;
    	std::cout << "failed" << std::endl;
		return -1;
	}

	// loop through all the results and bind to the first we can
	struct addrinfo *p;
	int sockfd = -1;
	for(p = servinfo; p != NULL; p = p->ai_next) {
		if ((sockfd = socket(p->ai_family, p->ai_socktype,
							 p->ai_protocol)) == -1) {
			perror("socket");
			continue;
		}

		if (::bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
			close(sockfd);
			perror("bind");
			continue;
		}

		if (fcntl(sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0)
		{
			perror("fcntl");
			close(sockfd);
	    	std::cout << "failed" << std::endl;
			return -1;
		}
		break;
	}

	if (p == NULL)
	{
		std::cout << "failed to find socket" << std::endl;
		std::cout << "failed" << std::endl;
		return -1;
	}

	freeaddrinfo(servinfo);

	std::cout << "success" << std::endl;
	return sockfd;
}

int AMNetworkUtils::openOutgoingSocket(const char *name, const char *hostname, const char *port, struct addrinfo &addr)
{
	std::cout << "opening " << name << " socket " << hostname << " on port " << port << " ..." << std::endl;

    struct addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo *servinfo;
    int rv;
    if ((rv = getaddrinfo(hostname, port, &hints, &servinfo)) != 0) {
    	std::cout << "getaddrinfo:"  <<  gai_strerror(rv) << std::endl;
    	std::cout << "failed" << std::endl;
        return -1;
    }

    // loop through all the results and make a socket
    struct addrinfo *p;
	int sockfd = -1;
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                             p->ai_protocol)) == -1) {
            perror("socket");
            continue;
        }

        break;
    }

    if (p == NULL) {
    	std::cout << "failed to find socket" << std::endl;
    	std::cout << "failed" << std::endl;
        return -1;
    }

    // return the address
    //TODO: this seems really dangerous - probably forgot how to do this right in c
    memcpy(addr.ai_addr, p->ai_addr, sizeof(struct sockaddr));
    addr.ai_addrlen = p->ai_addrlen;

    freeaddrinfo(servinfo); // all done with this structure

    std::cout << "success" << std::endl;
    return sockfd;
}

void AMNetworkUtils::getMacAddress(std::string channel, std::string &macAddress)
{
	// use the mac address for the vehicle ed
	// TODO: deal with failures (what if there is no ethernet?)
	char rv[20];
	std::string filename = "/sys/class/net/" + channel + "/address";
	std::cout << "mac address filename: " << filename << std::endl;
	FILE *fp = fopen(filename.c_str(), "r");
	fscanf(fp, "%s", rv);
	fclose(fp);
	macAddress = rv;
}

void AMNetworkUtils::closeSocket(int sockfp)
{
	close(sockfp);
}

} /* namespace brain_box */
