/*
 * ControlUtil.h
 *
 *  Created on: Mar 30, 2015
 *      Author: ubuntu
 */

#ifndef BRAIN_BOX_SRC_AM_SP_CONTROL_UTIL_H_
#define BRAIN_BOX_SRC_AM_SP_CONTROL_UTIL_H_

#include "math.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "brain_box_msgs/SPStatus.h"

/*
** Representation of vehicle state
*/
struct VehicleAttitude
{
  double roll;
  double pitch;
  double yaw;
  double throttle;
};

struct PIDGain {
	double P;
	double I;
	double D;
	double iMax;
	double iMin;
};

struct SetpointParams {
	double maxError;
	double maxSetpoint;
	double minSetpoint;
};

struct ControlParams {
	PIDGain	gain;
	SetpointParams setpoint;
};

struct XGains
{
  // Optional constructor for passing in values
  XGains(double p, double i, double d, double i_max, double i_min)
    : p_gain_(p),
      i_gain_(i),
      d_gain_(d),
      i_max_(i_max),
      i_min_(i_min)
  {}
  // Default constructor
  XGains() {}
  double p_gain_;  /**< Proportional gain. */
  double i_gain_;  /**< Integral gain. */
  double d_gain_;  /**< Derivative gain. */
  double i_max_;   /**< Maximum allowable integral term. */
  double i_min_;   /**< Minimum allowable integral term. */
};

struct Gimbal {
    double pitch;
    double yaw;
    double roll;
};

/*
 * Axis adjust
 */
#define ADJUST_VALUE	5

enum AxisMode:int {X=0, Y, Z, axisNum};
enum ParamMode:int {P=0, I, D, paramNum};
struct ButtonMapping {
	int 		button;
	AxisMode	axis;
	ParamMode	param;
};

extern const char* axisLabel[];
extern const char* paramLabel[];

enum class ControlTrigger {timer, targetPoseUpdate, vehiclePoseUpdate};

/*
 * Control modes of the smart pilot.
 */
enum class FLIGHT_MODE {LANDED=0, LAUNCHING, LOITER, TRACKING_TARGET, LANDING, TRAJECTORY, RTL, IDLE, WAYPOINT, TERRAIN, SEARCH, ROW, MANUAL};

// FCU control modes
enum FCUControlMode:int {NONE=0, ATTITUDE, POSITION, VELOCITY};

VehicleAttitude quaternionMsg2Att(geometry_msgs::Quaternion);

void testQuats();

#endif /* BRAIN_BOX_SRC_AM_SP_CONTROL_UTIL_H_ */
