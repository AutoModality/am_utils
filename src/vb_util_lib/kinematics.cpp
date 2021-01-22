/*
 * Kinematics.cpp
 *
 *  Created on: Apr 24, 2015
 *      Author: ubuntu
 */

#include <vb_util_lib/kinematics.h>
// #include <vb_util_lib/bag_logger.h> // TODO (dan) bag_logger
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

Kinematics::Kinematics() {
	_init();
}

Kinematics::Kinematics(ros::NodeHandle *n, std::string topic, std::string frame_id,
        bool pub_on_change, double pub_rate) :  nh_(n),
        topic_(topic), frame_id_(frame_id), pub_on_change_(pub_on_change), pub_rate_(pub_rate) {
    _init();
    initROS(n);
}

Kinematics::Kinematics(const geometry_msgs::PoseStamped& p) :
		frame_id_(p.header.frame_id) {
	_init();
	setPose(p);
}

Kinematics::Kinematics(const geometry_msgs::PoseStamped& p, const std::string id) :
		frame_id_(p.header.frame_id) {
	_init();
	setPose(p);
	object_id_ = id;
}

Kinematics::~Kinematics() {
	// TODO Auto-generated destructor stub
}

void Kinematics::_init() {
	velLinear_[0] = 0.0;
	velLinear_[1] = 0.0;
	velLinear_[2] = 0.0;
}

void Kinematics::initROS(ros::NodeHandle *n) {
    if (pub_on_change_ || pub_rate_ > 0.0)
    {
        publisher_ = true;
        pub_nh_ = n->advertise<nav_msgs::Odometry>(topic_.c_str(), 1000);
//        pub_nh_ = n->advertise<brain_box_msgs::VxKinematics>(topic_.c_str(), 1000);
    }
    else
    {
        publisher_ = false;
        sub_nh_ = n->subscribe<nav_msgs::Odometry>(topic_.c_str(), 1000,
                &Kinematics::updateCB, this);
    }

    if (pub_rate_ > 0)
    {
        pub_timer_ = n->createTimer(ros::Duration(pub_rate_), &Kinematics::publishTimerCB, this);
        pub_timer_.start();
    }

    if (isPublisher())
    {
    }

    // Set up the watchdog stuff
    // RVIZ debug display
    n->param("/Debug/kinematics_watchdog", enable_watchdog_, true);
    ROS_INFO("Kinematics: kinematics_watchdog[%s]", enable_watchdog_ ? "TRUE" : "FALSE");
    watchdog_timer_ = n->createTimer(ros::Duration(0.5), &Kinematics::watchdogTimerCB, this);
    if (!enable_watchdog_)
    {
    	watchdog_timer_.stop();
    }
    else
    {
    	watchdog_timer_.start();
    }


    ros_init_ = true;

    // RVIZ debug display
    n->param("/Debug/vizualization", rviz_debug_, false);
    if (rviz_debug_)
    {
        ROS_INFO("Kinematics: rivz enabled");
        rviz_uav_marker_pub_ = n->advertise<visualization_msgs::Marker>("/uav_marker", 1);
    }

    n->param("/Debug/Kinematics/velocity_debug", enable_velocity_debug_, false);

    // Initialize the update time
    setUpdateTime(ros::Time::now(), 5);
}

//void Kinematics::setPose(const std::string& source_frame, const ros::Time& update_time, const geometry_msgs::Pose& p, int debug) {
void Kinematics::setPose(const std::string& source_frame, const ros::Time& update_time, double X, double Y, double Z,
		double Qw, double Qx, double Qy, double Qz, int debug) {

	if (source_frame == frame_id_)
	{
		setPosition(update_time, X, Y, Z, debug);
		setAttitude(Qw, Qx, Qy, Qz);
//        ROS_DEBUG_STREAM_COND(debug_ && publisher_, "XXXXXXXX SAME FRAME = \n" << p);
	}
	else
	{
		geometry_msgs::TransformStamped ts;
		if (!transformer_.getTransform(frame_id_, source_frame, ts))
		{
			ROS_ERROR("Kinematics::setPose : can't find transform target[%s] source[%s]", source_frame.c_str(), frame_id_.c_str());
		}
		geometry_msgs::PoseStamped ts_pt, cur_pt;
		cur_pt.pose.position.x = X;
		cur_pt.pose.position.y = Y;
		cur_pt.pose.position.z = Z;
		cur_pt.pose.orientation.w = Qw;
		cur_pt.pose.orientation.x = Qx;
		cur_pt.pose.orientation.y = Qy;
		cur_pt.pose.orientation.z = Qz;
		cur_pt.header.frame_id = source_frame;
		cur_pt.header.stamp = update_time;
		// transform point
		tf2::doTransform(cur_pt, ts_pt, ts);
		setPosition(update_time, ts_pt.pose.position.x, ts_pt.pose.position.y, ts_pt.pose.position.z, debug);
		setAttitude(ts_pt.pose.orientation.w, ts_pt.pose.orientation.x, ts_pt.pose.orientation.y, ts_pt.pose.orientation.z);
//		ROS_DEBUG_STREAM_COND(debug_ && publisher_, "XXXXXXXX FROM POINT = \n" << cur_pt);
//		ROS_DEBUG_STREAM_COND(debug_ && publisher_, "XXXXXXXX TRANSFORM = \n" << ts);
//		ROS_DEBUG_STREAM_COND(debug_ && publisher_, "XXXXXXXX TO POINT = \n" << ts_pt);
	}

	ROS_DEBUG_COND(debug_ && publisher_, "XXXXXXXX FINAL XYZ[%0.2f  %0.2f  %0.2f]  RPY[%0.2f  %0.2f  %0.2f]",
			position_.x, position_.y, position_.z,
			attitude_.roll*(180.0/3.14159), attitude_.pitch*(180.0/3.14159), attitude_.yaw*(180.0/3.14159));

//    setUpdateTime(update_time);
}

void  Kinematics::getPose(geometry_msgs::Pose& p) {
    p.position.x = position_.x;
    p.position.y = position_.y;
    p.position.z = position_.z;
    p.orientation.x = orientation_.x();
    p.orientation.y = orientation_.y();
    p.orientation.z = orientation_.z();
    p.orientation.w = orientation_.w();

    return;
}


bool  Kinematics::getPoseStamped(std::string& target_frame_id, geometry_msgs::PoseStamped& ps) {

	geometry_msgs::TransformStamped ts;
	if (!transformer_.getTransform(target_frame_id, frame_id_, ts, true))
	{
		ROS_ERROR("Kinematics::getPoseStamped : No transform from[%s] to[%s]", frame_id_.c_str(), target_frame_id.c_str());
		return false;
	}

	ps.pose.position.x = ts.transform.translation.x;
	ps.pose.position.y = ts.transform.translation.y;
	ps.pose.position.z = ts.transform.translation.z;
	ps.pose.orientation.x = ts.transform.rotation.x;
	ps.pose.orientation.y = ts.transform.rotation.y;
	ps.pose.orientation.z = ts.transform.rotation.z;
	ps.pose.orientation.w = ts.transform.rotation.w;
	ps.header.stamp = update_time_;
	ps.header.frame_id = target_frame_id;

    return true;
}

void Kinematics::setPosition(const ros::Time& update_time, double x, double y, double z, int debug) {
	checkPosition(x, y, z, debug);
	setInitialized(POSE_STATE::XYZ);

	// Calculate speed
	double delta_t = update_time.toSec() - update_time_.toSec();

	if (update_cnt_ <= 0 || delta_t <= 0.0)
	{
		position_.x = x;
		position_.y = y;
		position_.z = z;

		setLinearVelocity(0.0, 0.0, 0.0);

		setUpdateTime(update_time, 2);

		return;
	}

	double B = 1.0 - speed_filter_;

	Eigen::Vector3d vel;

	vel[0] = (x - position_.x) / delta_t;
	vel[1] = (y - position_.y) / delta_t;
	vel[2] = (z - position_.z) / delta_t;

	vel = (vel * speed_filter_) + (B * velLinear_);
	velLinear_ = vel;

	position_.x = x;
	position_.y = y;
	position_.z = z;

	setUpdateTime(update_time, 3);

	if (enable_velocity_debug_)
	{
		Eigen::Vector3d vel_debug;
		getLinearVelocity(drone_ENU, vel_debug);
//		ROS_INFO("Kinematics Velocity standard[%s :  %0.3f %0.3f, %0.3f]   body[%s :  %0.3f %0.3f, %0.3f]",
//				frame_id_.c_str(), velLinear_[0], velLinear_[1], velLinear_[2],
//				drone_ENU, vel_debug[0], vel_debug[1], vel_debug[2]);
	}
}

#define MAX_POS_ERR 1.0

void Kinematics::checkPosition(double x, double y, double z, int debug) {
	if (debug >= 0)
	{
		if (fabs(position_.x - x) > MAX_POS_ERR ||
			fabs(position_.y - y) > MAX_POS_ERR ||
		    fabs(position_.z - z) > MAX_POS_ERR)
		{
			ROS_ERROR(">>> Kinematics hyperspace[%d] X[%0.2f : %0.2f] Y[%0.2f : %0.2f] Z[%0.2f : %0.2f] ",
					debug, position_.x, x, position_.y, y, position_.z, z);
		}
	}

	return;
}

//void Kinematics::setAttitude(const VehicleAttitude& va, bool yaw_corrected) {
//	setInitialized(POSE_STATE::RPY);
//	if (yaw_corrected)
//	{
//		setInitialized(POSE_STATE::YAW_CORRECTED);
//	}
//	attitude_ = va;
//	AM::RPY_to_Quaternion(orientation_, attitude_.roll, attitude_.pitch, attitude_.yaw);
//}
//
void Kinematics::setAttitude(double roll, double pitch, double yaw, bool yaw_corrected, double throttle) {
	setInitialized(POSE_STATE::RPY);
	if (yaw_corrected)
	{
		setInitialized(POSE_STATE::YAW_CORRECTED);
	}
	attitude_.roll = roll;
	attitude_.pitch = pitch;
	attitude_.yaw = yaw;
	attitude_.throttle = throttle;
	AM::RPY_to_Quaternion(orientation_, attitude_.roll, attitude_.pitch, attitude_.yaw);
}

void Kinematics::setOrientation(double w, double x, double y, double z, bool yaw_corrected) {
	setInitialized(POSE_STATE::RPY);
	if (yaw_corrected)
	{
		setInitialized(POSE_STATE::YAW_CORRECTED);
	}
    orientation_ = Eigen::Quaterniond(w, x, y, z);
    Eigen::Vector3d rpy = AM::quaternion_to_RPY(orientation_);
    attitude_.roll = rpy[0];
    attitude_.pitch = rpy[1];
    attitude_.yaw = rpy[2];
    attitude_.throttle = 0;
}

void Kinematics::setYaw(double yaw, bool corrected) {
	attitude_.yaw = yaw;
	AM::RPY_to_Quaternion(orientation_, attitude_.roll, attitude_.pitch, attitude_.yaw);
	setInitialized(POSE_STATE::YAW);
	if (corrected)
	{
		setInitialized(POSE_STATE::YAW_CORRECTED);
	}
}

void Kinematics::updateVelocity(double x, double y, double z, double t,
		int debug) {
}

void Kinematics::setVelocity(const geometry_msgs::Vector3& linear,
        const geometry_msgs::Vector3& angular) {
    setLinearVelocity(linear);
    setAngularVelocity(angular);
}

void Kinematics::setVelocity(const Eigen::Vector3d& linear,
        const Eigen::Vector3d& angular) {
    setLinearVelocity(linear);
    setAngularVelocity(angular);
}

bool Kinematics::getLinearVelocity(std::string target_frame_id, Eigen::Vector3d& velocity) {

	geometry_msgs::TransformStamped ts;
	if (!transformer_.getTransform(target_frame_id, frame_id_, ts, true))
	{
		ROS_ERROR("Kinematics::getLinearVelocity : No transform from[%s] to[%s]", frame_id_.c_str(), target_frame_id.c_str());
		return false;
	}

	Eigen::Quaterniond rot;
	rot.x() = ts.transform.rotation.x;
	rot.y() = ts.transform.rotation.y;
	rot.z() = ts.transform.rotation.z;
	rot.w() = ts.transform.rotation.w;

	velocity = rot * velLinear_;

//	ps.pose.position.x = ts.transform.translation.x;
//	ps.pose.position.y = ts.transform.translation.y;
//	ps.pose.position.z = ts.transform.translation.z;
//	ps.pose.orientation.x = ts.transform.rotation.x;
//	ps.pose.orientation.y = ts.transform.rotation.y;
//	ps.pose.orientation.z = ts.transform.rotation.z;
//	ps.pose.orientation.w = ts.transform.rotation.w;
//	ps.header.stamp = update_time_;
//	ps.header.frame_id = target_frame_id;

    return true;
}

void Kinematics::setAngularVelocity(double x, double y, double z) {
	velAngular_[0] = x;
	velAngular_[1] = y;
	velAngular_[2] = z;
}

void Kinematics::setLinearVelocity(double x, double y, double z) {
	velLinear_[0] = x;
	velLinear_[1] = y;
	velLinear_[2] = z;
}

void Kinematics::getOdometry(nav_msgs::Odometry& ms) {
	ms.pose.pose.position.x = position_.x;
	ms.pose.pose.position.y = position_.y;
	ms.pose.pose.position.z = position_.z;

	ms.pose.pose.orientation.w = orientation_.w();
	ms.pose.pose.orientation.x = orientation_.x();
	ms.pose.pose.orientation.y = orientation_.y();
	ms.pose.pose.orientation.z = orientation_.z();

	ms.twist.twist.linear.x = velLinear_[0];
	ms.twist.twist.linear.y = velLinear_[1];
	ms.twist.twist.linear.z = velLinear_[2];

	ms.twist.twist.angular.x = 0.0;
	ms.twist.twist.angular.y = 0.0;
	ms.twist.twist.angular.z = 0.0;

	ms.header.stamp = update_time_;
	ms.header.frame_id = frame_id_;
	ms.child_frame_id = drone_ENU;

	// Set the initialized attributes
	ms.pose.covariance[0] = pose_initialized_;
	return;
}


  /*
void Kinematics::publishTransform() {
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(position.x, position.y, position.z) );
	tf::Quaternion q;
	tf::quaternionMsgToTF(orientation_, q);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));
}
  */

AM::Point Kinematics::convertRFUtoENUoff(double right, double front,
        double up, double yaw) {
    AM::Point p;

    double u_off = up;
    double n_off = front;
    double e_off = -right;

    double total_xy = sqrt((e_off*e_off)+(n_off*n_off));
    double ang_off;
    if (total_xy != 0)
    {
        ang_off = atan2(e_off, n_off);
        double ang = ang_off + yaw;
        n_off = total_xy * cos(ang);
        e_off = -total_xy * sin(ang);
    }

    p.x = e_off;
    p.y = n_off;
    p.z = u_off;

    return p;
}

void Kinematics::convertRFUtoENU(Kinematics& RFU, double yaw, Kinematics* ENU) {
    //variables associated with the camera view
    double Xc, Yc, Dc, Ac;
    // Variables associated with the current vehicle state (i.e. yaw)
    double Ay;
    // Calculated Variables associated with the calculated target position in ENU
    double Xt, Yt, At;

    Xc = RFU.position_.x;
    Yc = RFU.position_.y;
    Dc = sqrt((Xc*Xc) + (Yc*Yc));
    Ay = yaw + PI_2;

    if (Dc > 0)
    {
        Ac = asin(-Xc/Dc);
        At = Ay + Ac;
        Xt = Dc * cos(At);
        Yt = Dc * sin(At);
    }
    else
    {
        Xt = 0;
        Yt = 0;
    }

    // update the target ENU state
    ENU->position_.x = Xt;
    ENU->position_.y = Yt;
    ENU->position_.z = RFU.position_.z;
    ENU->setAttitude(RFU.getAttitude());
}

AM::Point  Kinematics::convertFLUtoENUoff(double front, double left, double up, double yaw) {
    //variables associated with the camera view
    double Xc, Yc, Dc, Ac;
    // Variables associated with the current vehicle state (i.e. yaw)
    double Ay;
    // Calculated Variables associated with the calculated target position in ENU
    double Xt, Yt, At;
    Xc = front;
    Yc = left;
    Dc = sqrt((Xc*Xc) + (Yc*Yc));
    Ay = yaw;
    //    printf("FLUtoNED  from %0.4f, %0.4f, %0.4f\n", Xc, Yc, yaw);

    if (Dc > 0)
    {
//        Ac = asin(Yc/Dc);
        Ac = atan2(Yc, Xc);
        At = Ay + Ac;
        printf("  FLU_toENUoff  Angles Ay[%0.2f] Ac[%0.2f] At[%0.2f]\n",
               (Ay/3.14159)*180.0, (Ac/3.14159)*180.0, (At/3.14159)*180.0);
//        printf("FLUtoNED  Xc[%0.4f] Yc[%0.4f] Dc[%0.4f] Ay[%0.4f] Ac[%0.4f], At[%0.4f]\n", Xc, Yc, Dc, Ay, Ac, At);
        Xt = Dc * cos(At);
        Yt = Dc * sin(At);
    }
    else
    {
        Xt = 0;
        Yt = 0;
    }

//    printf("FLUtoNED  to %0.4f, %0.4f, %0.4f, %0.4f\n", Xt, Yt, Dc, At);

    AM::Point p;
    p.x = Xt;
    p.y = Yt;
    p.z = -up;
    return p;
}

//TODO LOC FIX
void Kinematics::convertFLUtoENU(Kinematics& FLU, Kinematics& ref_ENU, Kinematics* ENU) {
    AM::Point p = convertFLUtoENUoff(FLU.position_.x, FLU.position_.y,
            FLU.position_.z, ref_ENU.attitude_.yaw);
    ENU->position_.x = p.x + ref_ENU.position_.x;
    ENU->position_.y = p.y + ref_ENU.position_.y;
    ENU->position_.z = p.z + ref_ENU.position_.z;
//    ENU->position_.x = p.x;
//    ENU->position_.y = p.y;
//    ENU->position_.z = p.z;
    ENU->setAttitude(FLU.getAttitude());  // TODO LOC FIX ATTITUDE
}

void Kinematics::sendKinematics() {
    if (!ros_init_ || !publisher_)
    {
        return;
    }

    nav_msgs::Odometry ms;

    getOdometry(ms);
//print(">>>>>>> SENDING KINEMATICS: ", true);
    pub_nh_.publish(ms);
    // LOG_MSG(topic_, ms, 2); // TODO (dan) bag_logger
}

void Kinematics::updateCB(const nav_msgs::Odometry::ConstPtr& msg) {
    // LOG_MSG(topic_+"-R", msg, 2); // TODO (dan) bag_logger
    setUpdated(true);
	setPosition(msg->header.stamp, msg->pose.pose.position);
	setAttitude(msg->pose.pose.orientation);
    setLinearVelocity(msg->twist.twist.linear);
    //setUpdateTime(msg->header.stamp, 4);
    // Set the initialization flags
    pose_initialized_ = msg->pose.covariance[0];

//    update_time_ = ros::Time::now();

//    setOdometry(msg);
//print(">>>>>>> RECEIVING KINEMATICS: ", true);

    if (callback_ != NULL)
    {
        callback_(this);
    }

    if (log_enabled_) {
        msg_ = *msg;
    }
}

void Kinematics::watchdogTimerCB(const ros::TimerEvent& te) {
	ros::Duration d = ros::Time::now() - update_time_;
    update_stale_ = last_update_cnt_ == update_cnt_;
    if (update_stale_)
    {
    	ROS_ERROR("ERROR: KINEMATICS WATCHDOG UPDATE STALE cnt[%ld]", update_cnt_);
    }
    last_update_cnt_ = update_cnt_;
}

void Kinematics::doUpdate() {
  //        ROS_INFO("FCU STATUS: checkUpdate IN");
    setUpdated(true);
    if (pub_on_change_)
    {
//        print("K_UP", true);
        sendKinematics();
    }
    if (callback_ != NULL)
    {
        callback_(this);
    }
}

void Kinematics::test() {
	double roll = 1.0;
	double pitch = 1.1;
	double yaw = 1.2;
	double throttle = 3.1;
	geometry_msgs::PoseStamped p;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = Asset_ENU;
	p.pose.position.x = 5.1;
	p.pose.position.y = 5.2;
	p.pose.position.z = 5.3;
	p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	geometry_msgs::Vector3 vl;
	vl.x = 10.0;
	vl.y = 10.1;
	vl.z = 10.2;
	geometry_msgs::Vector3 van;
	van.x = 20.0;
	van.y = 20.1;
	van.z = 20.2;

	printf("Kinematics testing roll:%f, pitch:%f, yaw:%f, throttle:%f\n",
			roll, pitch, yaw, throttle);
	printf("          Angular velocity x:%f, y:%f, z:%f\n", vl.x, vl.y, vl.z);
	printf("          Linear velocity x:%f, y:%f, z:%f\n", van.x, van.y, van.z);

	printf("Kinematics: setPosition - x,y,z\n");
	setPosition(ros::Time::now(), p.pose.position.x, p.pose.position.y, p.pose.position.z);
	print("TEST");

	printf("Kinematics: setPosition - position\n");
	setPosition(ros::Time::now(), p.pose.position);
	print("TEST");

	printf("Kinematics: setPose\n");
	setPose(p);
	print("TEST");

	printf("Kinematics: setAttitude - va\n");
	VehicleAttitude va;
	va.roll = roll;
	va.pitch = pitch;
	va.yaw = yaw;
	va.throttle = throttle;
	setAttitude(va);
	print("TEST");

	printf("Kinematics: setAttitude - RPY\n");
	setAttitude(roll, pitch, yaw, throttle);
	print("TEST");

	printf("Kinematics: setAttitude - quaternion\n");
	setAttitude(p.pose.orientation);
	print("TEST");

	printf("Kinematics: setVelocity - linear, angular\n");
	setVelocity(vl, van);
	print("TEST");

	printf("Kinematics: setAngularVelocity - angular\n");
	setAngularVelocity(van);
	print("TEST");

	printf("Kinematics: setAngularVelocity - x, y, z\n");
	setAngularVelocity(van.x, van.y, van.z);
	print("TEST");
}

//const AM::Point& Kinematics::getGlobalPosition() {
//	global_position_.x = position_.x - origin_.x;
//	global_position_.y = position_.y - origin_.y;
//	global_position_.z = position_.x - origin_.z;
//
//	return global_position_;
//}
//
void Kinematics::print(std::string prefix, bool loc_only) {
	printf("%s: Kinematics position: x:%0.3f, y:%f0.3, z:%f0.3, yawD:%f0.2\n",
			prefix.c_str(), position_.x, position_.y, position_.z, attitude_.yaw*(180.0/3.14159));

	if (!loc_only)
	{
	    printf("           orientation: x:%f, y:%f, z:%f, w:%f\n", orientation_.x(), orientation_.y(), orientation_.z(), orientation_.w());
	    printf("           Attitude: rollD:%f, pitchD:%f, yawD:%f, throttle:%f\n",
	            attitude_.roll*(180.0/3.14159), attitude_.pitch*(180.0/3.14159), attitude_.yaw*(180.0/3.14159), attitude_.throttle);
	    printf("           linear veloctiy: x:%f, y:%f, z:%f\n", velLinear_[0], velLinear_[1], velLinear_[2]);
	    printf("           Angular veloctiy: x:%f, y:%f, z:%f\n", velAngular_[0], velAngular_[1], velAngular_[2]);
	}
}

void Kinematics::publishMarker() {
    if (!rviz_debug_)
    {
        return;
    }
    if (rviz_point_cnt > 0)
    {
        visualization_msgs::Marker line;
        geometry_msgs::Point p;

        line.header.frame_id = frame_id_;
        line.header.stamp = ros::Time::now();
        line.ns = "uav";
        line.id = rviz_point_cnt;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        line.scale.x = 0.075;

        // Set the color -- be sure to set alpha to something non-zero!
        line.color.r = 1.0f;
        line.color.g = 0.0f;
        line.color.b = 0.0f;
        line.color.a = 1.0;

        line.lifetime = ros::Duration();

        p.x = marker.pose.position.x;
        p.y = marker.pose.position.y;
        p.z = marker.pose.position.z;
        line.points.push_back(p);
        p.x = position().x;
        p.y = position().y;
        p.z = position().z;
        line.points.push_back(p);

        rviz_uav_marker_pub_.publish(line);
    }

    // Send the current location to RVIZ
    marker.header.frame_id = frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "uav";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = position().x;
    marker.pose.position.y = position().y;
    marker.pose.position.z = position().z;
    // translate roll, pitch and yaw into a Quaternion
    tf::Quaternion q;
    //        q.setRPY(3.14159, 0.0, 0.0);
    //        geometry_msgs::Quaternion qt;
    //        tf::quaternionTFToMsg(q, qt);
    //        printf("******* x[%0.7f] y[%0.7f] z[%0.7f] w[%0.7f] \n", qt.x, qt.y, qt.z, qt.w);
    q.setRPY(getAttitude().roll,
            getAttitude().pitch,
            getAttitude().yaw);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);

    marker.pose.orientation.x = odom_quat.x;
    marker.pose.orientation.y = odom_quat.y;
    marker.pose.orientation.z = odom_quat.z;
    marker.pose.orientation.w = odom_quat.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 0.4;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.9;

    marker.lifetime = ros::Duration();
    rviz_uav_marker_pub_.publish(marker);
    rviz_point_cnt++;
}
