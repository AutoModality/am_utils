/*
 * operator_msg_type.h
 *
 *  Created on: Nov 24, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_OPERATOR_MSG_TYPE_H_
#define VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_OPERATOR_MSG_TYPE_H_

#include <string>

/**
 * Constants used in OperatorMsg.msg
 */

enum class OPERATOR_MSG_TYPE {INFO=0, WARNING=1, OP_ERROR=2, CRITICAL=3, SYSTEM=4, COMMAND=5};

// Define strings corresponding to types
extern std::string const OPERATOR_MSG_STRING[];

#define OP_MSG_STR(t) OPERATOR_MSG_STRING[static_cast<int>(t)]

#endif /* VISBOX_PACKAGES_COMMON_MESSAGES_INCLUDE_BRAIN_BOX_MSGS_OPERATOR_MSG_TYPE_H_ */
