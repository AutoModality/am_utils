#ifndef BRAIN_BOX_MSGS_INCLUDE_BRAIN_BOX_MSGS_MESSAGE_UTILS_CPP_
#define BRAIN_BOX_MSGS_INCLUDE_BRAIN_BOX_MSGS_MESSAGE_UTILS_CPP_

#include <string>

class MessageUtil
{
public:
	static std::string statusIntToString(int status);
	static std::string statusIntToChar(int status);
};


#endif /* BRAIN_BOX_MSGS_INCLUDE_BRAIN_BOX_MSGS_MESSAGE_UTILS_CPP_ */
