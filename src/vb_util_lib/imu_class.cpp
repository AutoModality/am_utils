/*
 * ImuInfuser.cpp
 *
 *  Created on: May 8, 2018
 *      Author: Alireza Janani
 */


#include <vb_util_lib/imu_class.h>


namespace am
{

ImuClass::ImuClass(ros::NodeHandle &nh, const std::string &imu_topic, int imu_pot_size): imu_pot_size_(imu_pot_size), imu_cnt_(-1)
{
	nh_ = nh;
	px4_imu_sub_= nh_.subscribe<sensor_msgs::Imu>(imu_topic, 100, &ImuClass::imuCB, this);
	imu_pot_ = std::vector<sensor_msgs::Imu>(imu_pot_size);
}

ImuClass::ImuClass(ros::NodeHandle &nh, int imu_pot_size): imu_cnt_(-1)
{
	nh_ = nh;
	dji_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(am_topics::DJI_SDK_IMU, 100, &ImuClass::imuCB, this);
	px4_imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(am_topics::MAVROS_IMU_DATA, 100, &ImuClass::imuCB, this);
	imu_pot_size_ = imu_pot_size;
	imu_pot_ = std::vector<sensor_msgs::Imu>(imu_pot_size_);
}
ImuClass::ImuClass(): imu_cnt_(-1)
{
	imu_pot_size_ = 0;
}
ImuClass::ImuClass(int imu_pot_size): imu_cnt_(-1)
{
	imu_pot_size_ = imu_pot_size;
}
ImuClass::~ImuClass()
{

}
void ImuClass::imuCB(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
	//find the index
	imu_cnt_++;
	if(imu_cnt_ >= imu_pot_size_)
	{
		imu_cnt_ = 0;
	}
	//insert
	imu_pot_[imu_cnt_] = *imu_msg;
}
void ImuClass::addImu(const sensor_msgs::Imu &imu)
{
	//find the index
	imu_cnt_++;
	if(imu_cnt_ >= imu_pot_size_)
	{
		imu_cnt_ = 0;
	}
	//insert
	imu_pot_[imu_cnt_] = imu;
}
int ImuClass::getImu(std::vector<sensor_msgs::Imu> &res, const ros::Time &t, double time_difference)
{
	/*
	 * imu_pot_ manipulation might be carried out asynchronously and at higher rate.
	 * For example, imu messages are published at 400 Hz whereas lidar pointclouds
	 * are published at 40 Hz. This causes the imu_pot_ to be changed while iterating
	 * through the messages. For prevention, a local copy of the original pot should be
	 * created.
	 */
	for(int i = 0; i < imu_pot_.size(); i++)
	{
		double time_diff =  t.toSec() - imu_pot_[i].header.stamp.toSec();
		if(abs(time_diff) <= time_difference && time_diff < 0)
		{
			res.push_back(imu_pot_[i]);
		}
	}
	if(res.size() > 0) return 0;
	else return -1;
}
//Returns the Imu closest to time
int ImuClass::getImu(sensor_msgs::Imu &res, const ros::Time &t)
{
	double min_time_diff = 1000000.00;
	int min_imu_index = -1;
	/*
	 * imu_pot_ manipulation might be carried out asynchronously and at higher rate.
	 * For example, imu messages are published at 400 Hz whereas lidar pointclouds
	 * are published at 40 Hz. This causes the imu_pot_ to be changed while iterating
	 * through the messages. For prevention, a local copy of the original pot should be
	 * created.
	 */
	for(int i = 0; i < imu_pot_.size(); i++)
	{
		double time_diff =  abs(t.toSec() - imu_pot_[i].header.stamp.toSec());
		if(time_diff <= min_time_diff)
		{
			min_time_diff = time_diff;
			min_imu_index = i;

		}
	}
	if(min_imu_index < 0)
	{
		return -1;
	}

	res = imu_pot_[min_imu_index];
	return 0;
}

//return the latest IMU
int ImuClass::getImu(sensor_msgs::Imu &res)
{
	if(imu_cnt_ < 0)
	{
		return -1;
	}
	res = imu_pot_[imu_cnt_];
	return 0;
}
void ImuClass::setup(int imu_pot_size)
{
	imu_pot_size_ = imu_pot_size;
}
int ImuClass::transform(geometry_msgs::Point32 &p, const ros::Time &t, int infuse_axis)
{
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		return -1;
	}
	double roll, pitch, yaw;
	applyAxis(imu, infuse_axis, roll, pitch, yaw);
	am::Rotate R(roll, pitch, yaw);

	if(roll != 0.0)R.rotate(p.x, p.y, p.z, 'X');
	if(pitch != 0.0)R.rotate(p.x, p.y, p.z, 'Y');
	if(yaw != 0.0)R.rotate(p.x, p.y, p.z, 'Z');
	return 0;
}
int ImuClass::transform(cv::Point &p, const ros::Time &t, int infuse_axis)
{
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		return -1;
	}

	double roll, pitch, yaw;
	applyAxis(imu, infuse_axis, roll, pitch, yaw);
	am::Rotate R(roll, pitch, yaw);
	//float z =0.0;
	//if(roll != 0.0)R.rotate(p.x, p.y, z, 'X');
	//if(pitch != 0.0)R.rotate(p.x, p.y, z, 'Y');
	return 0;
}
int ImuClass::transform(nav_msgs::Odometry &odom, const ros::Time &t, std::vector<double> &rpy, int infuse_axis)
{
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		printf("ImuClass::transform() -> Getting IMU Error \n");
		return -1;
	}
	double roll, pitch, yaw;
	applyAxis(imu, infuse_axis, roll, pitch, yaw);
	am::Rotate R(roll, pitch, yaw);
	//printf("%f, %f, %f\n", roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

	am::Rotate::getRPY(imu.orientation, roll, pitch, yaw);
	rpy.push_back(roll);
	rpy.push_back(pitch);
	rpy.push_back(yaw);
	float x = odom.pose.pose.position.x;
	float y = odom.pose.pose.position.y;
	float z = odom.pose.pose.position.z;

	if(roll != 0.0)R.rotate(x, y, z, 'X');
	if(pitch != 0.0)R.rotate(x, y, z, 'Y');
	if(yaw != 0.0)R.rotate(x, y, z, 'Z');

	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = z;

	return 0;
}
int ImuClass::transform(float &x, float &y, float &z, const ros::Time &t, int infuse_axis)
{
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		return -1;
	}
	double roll, pitch, yaw;
	applyAxis(imu, infuse_axis, roll, pitch, yaw);
	am::Rotate R(roll, pitch, yaw);

	if(roll != 0.0)R.rotate(x, y, z, 'X');
	if(pitch != 0.0)R.rotate(x, y, z, 'Y');
	if(yaw != 0.0)R.rotate(x, y, z, 'Z');

	return 0;
}

int ImuClass::transform(int &x, int &y, int &z, const ros::Time &t, int infuse_axis)
{
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		return -1;
	}
	double roll, pitch, yaw;
	applyAxis(imu, infuse_axis, roll, pitch, yaw);
	am::Rotate R(roll, pitch, yaw);

	float fx = (float) x, fy = (float) y, fz = (float) z;

	if(roll != 0.0)R.rotate(fx, fy, fz, 'X');
	if(pitch != 0.0)R.rotate(fx, fy, fz, 'Y');
	if(yaw != 0.0)R.rotate(fx, fy, fz, 'Z');

	x = fx;
	y = fy;
	z = fz;
	return 0;
}
int ImuClass::transform(sensor_msgs::PointCloud &pc, const ros::Time &t, double time_difference, int infuse_axis)
{
	std::vector<sensor_msgs::Imu> imus;
	if(getImu(imus, t, time_difference) < 0)
	{
		return -1;
	}

	std::vector<int> lidar_sectors(imus.size());
	int dividers = pc.points.size() / imus.size();
	lidar_sectors[0] = dividers;

	int index = 0;
	while(index < lidar_sectors.size())
	{
		//am::Rotate R(imus[index].orientation);
		double roll, pitch, yaw;
		applyAxis(imus[index], infuse_axis, roll, pitch, yaw);
		am::Rotate R(roll, pitch, yaw);

		int lowBound = index * lidar_sectors[0];
		int highBound = (index + 1) * lidar_sectors[0];
		if(index == lidar_sectors.size() - 1)
		{
			highBound = pc.points.size();
		}
		for(int i = lowBound; i < highBound; i++)
		{
			if(roll != 0.0)R.rotate(pc.points[i].x, pc.points[i].y, pc.points[i].z, 'X');
			if(pitch != 0.0)R.rotate(pc.points[i].x, pc.points[i].y, pc.points[i].z, 'Y');
			if(yaw != 0.0)R.rotate(pc.points[i].x, pc.points[i].y, pc.points[i].z, 'Z');
		}
		index++;
	}
	return 0;
}
int ImuClass::transform(sensor_msgs::PointCloud &pc, const ros::Time &t, int infuse_axis)
{
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		return -1;
	}
	double roll, pitch, yaw;
	applyAxis(imu, infuse_axis, roll, pitch, yaw);

	//ROS_INFO("IMU R: %f, P: %f, Y: %f", roll, pitch, yaw);
	am::Rotate R(roll, pitch, yaw);

	for(auto &p : pc.points)
	{
		if(roll != 0.0)R.rotate(p.x, p.y, p.z, 'X');
		if(pitch != 0.0)R.rotate(p.x, p.y, p.z, 'Y');
		if(yaw != 0.0)R.rotate(p.x, p.y, p.z, 'Z');
	}
	return 0;
}
int ImuClass::transform(cv::Mat &img, const ros::Time &t)
{
	/*********Rotation Matrix Acquisition Begin*********/
	sensor_msgs::Imu imu;
	if(getImu(imu, t) < 0)
	{
		return -1;
	}
	cv::Mat rotation_m = ImuClass::getRotation(imu);

	/*********Perspective Transformation Begin*********/
	cv::Mat img_t;
	cv::warpPerspective(img, img_t, rotation_m, img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

	img = img_t;
}

//Transform PointCloud2
int ImuClass::transform(const sensor_msgs::PointCloud2 &src, sensor_msgs::PointCloud2 &res, const ros::Time &t)
{
	sensor_msgs::PointCloud pc;
	sensor_msgs::convertPointCloud2ToPointCloud(src, pc);
	transform(pc,t);
	sensor_msgs::convertPointCloudToPointCloud2(pc, res);
	res.header = src.header;
	return 0;
}
void ImuClass::applyAxis(sensor_msgs::Imu &src, int axis, double &roll, double &pitch, double &yaw)
{
	am::Rotate::getRPY(src.orientation, roll, pitch, yaw);
	switch(axis)
	{
	case AXIS::RPY:
	{
		return;
	}
	case AXIS::ROLL:
	{
		pitch = 0.0;
		yaw = 0.0;
		return;
	}
	case AXIS::ROLL_PITCH:
	{
		yaw = 0.0;
		return;
	}
	case AXIS::ROLL_YAW:
	{
		pitch = 0.0;
		return;
	}
	case AXIS::PITCH:
	{
		roll = 0.0;
		yaw = 0.0;
		return;
	}
	case AXIS::PITCH_YAW:
	{
		roll = 0.0;
		return;
	}
	case AXIS::YAW:
	{
		roll = 0.0;
		pitch = 0.0;
		return;
	}
	}
}
void ImuClass::normalize(sensor_msgs::Imu &imu)
{

}
cv::Mat ImuClass::getRotation(sensor_msgs::Imu &imu, bool doNormalization)
{
	cv::Mat res(3, 3, CV_64FC1);

	if(doNormalization)
	{
		normalize(imu);
	}

	res.at<float>(0,0) = 1 - 2*pow(imu.orientation.y,2) - 2*pow(imu.orientation.z,2);
	res.at<float>(0,1) = 2*imu.orientation.x*imu.orientation.y + 2*imu.orientation.z*imu.orientation.w;
	res.at<float>(0,2) = 2*imu.orientation.x*imu.orientation.z - 2*imu.orientation.y*imu.orientation.w;

	res.at<float>(1,0) = 2*imu.orientation.x*imu.orientation.y - 2*imu.orientation.z*imu.orientation.w;
	res.at<float>(1,1) = 1 - 2*pow(imu.orientation.x,2) - 2*pow(imu.orientation.z,2);
	res.at<float>(1,2) = 2*imu.orientation.y*imu.orientation.z + 2*imu.orientation.x*imu.orientation.w;

	res.at<float>(2,0) = 2*imu.orientation.x*imu.orientation.z + 2*imu.orientation.y*imu.orientation.w;
	res.at<float>(2,1) = 2*imu.orientation.y*imu.orientation.z - 2*imu.orientation.x*imu.orientation.w;
	res.at<float>(2,2) = 1 - 2*pow(imu.orientation.x,2) - 2*pow(imu.orientation.y,2);

	return res;
}
}



