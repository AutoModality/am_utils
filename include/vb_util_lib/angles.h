/*
 * ftp_requset.h
 *
 *  Created on: Mar 13, 2016
 *      Author: ubuntu
 */

#ifndef INCLUDE_VB_UTIL_LIB_ANGLES_H_
#define INCLUDE_VB_UTIL_LIB_ANGLES_H_

namespace am
{

class Angles
{
public:
	static int compare(double ang1, double ang2);
	static double getDiffSigned(double ang1, double ang2);
	static double getDiff(double ang1, double ang2);
        static double getOrderedDiff(double ang1, double ang2);
	static bool isBetween(double target, double left_angle, double right_angle, double & clamped);

private:
};

}
#endif /* INCLUDE_VB_UTIL_LIB_ANGLES_H_ */
