/*init_roll_ = init_r;
 * ftp_requset.h
 *
 *  Created on: Mar 13, 2016
 *      Author: ubuntu
 */

#ifndef INCLUDE_VB_UTIL_LIB_ANGLE_AB_FILTER_H_
#define INCLUDE_VB_UTIL_LIB_ANGLE_AB_FILTER_H_

#include <geometry_msgs/msg/quaternion.hpp>
#include <vb_util_lib/rotate.h>

namespace am
{

class AngleAlphaBetaFilter
{
public:
	AngleAlphaBetaFilter(double weight, double init_r = 0.0, double init_p = 0.0, double init_y = 0.0);
	~AngleAlphaBetaFilter();
	void setWeight(double weight)
	{
		weight_ = weight;
	}

	void initialize(const geometry_msgs::msg::Quaternion &q);

	void initialize(double r, double p, double y);

	double filter(double new_angle);

	bool updateIfOk(double angle, double &filtered_angle);

	bool filter(geometry_msgs::msg::Quaternion &q);

	double getAngle()
	{
		return old_angle_;
	}

private:
	double weight_ = 0.5;
	double old_angle_ = 0.0;
	bool data_is_set_ = false;

	double init_roll_ = {0.0};
	double init_pitch_ = {0.0};
	double init_yaw_ = {0.0};
};

}
#endif /* INCLUDE_VB_UTIL_LIB_ANGLE_AB_FILTER_H_ */
