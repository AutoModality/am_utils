/*
 * am_network_utils.h
 *
 *  Created on: Jun 2, 2015
 *      Author: dan
 */

#ifndef AM_UTILS_SRC_AM_NETWORK_UTILS_H_
#define AM_UTILS_SRC_AM_NETWORK_UTILS_H_

#include <sys/socket.h>
#include <string>
#include <unistd.h>

namespace am {

class AMNetworkUtils {
public:
	AMNetworkUtils();
	virtual ~AMNetworkUtils();
	static void *get_in_addr(struct sockaddr *sa);
	static void printSource(struct sockaddr_storage their_addr, socklen_t addr_len, long bytesReceived);
	static int openIncomingSocket(const char *name, const char *port);
	static int openOutgoingSocket(const char *name, const char *hostname, const char *port, struct addrinfo &addr);
	static void getMacAddress(std::string channel, std::string &macAddress);
	static void closeSocket(int sockfp);
};

} /* namespace am */

#endif /* AM_UTILS_SRC_AM_NETWORK_UTILS_H_ */
