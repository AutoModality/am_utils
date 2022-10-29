/*
 * am_network_utils.h
 *
 *  Created on: Jun 2, 2015
 *      Author: dan
 */

#ifndef BRAIN_BOX_MSGS_SRC_AM_NETWORK_UTILS_H_
#define BRAIN_BOX_MSGS_SRC_AM_NETWORK_UTILS_H_

#include <sys/socket.h>
#include <unistd.h>

namespace brain_box {

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

} /* namespace brain_box */

#endif /* BRAIN_BOX_MSGS_SRC_AM_NETWORK_UTILS_H_ */
