/*
 * time_util.h
 *
 *  Created on: Apr 9, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TIME_UTIL_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TIME_UTIL_H_

#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

class TimeUtil
{
public:
	static std::string printTime(boost::posix_time::ptime timestamp);
};


#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_TIME_UTIL_H_ */
