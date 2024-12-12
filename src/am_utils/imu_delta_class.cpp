#include <am_utils/imu_delta_class.h>

namespace am
{
ImuDelta::ImuDelta(bool use_original_roll, bool use_original_pitch) : use_original_roll_(use_original_roll), use_original_pitch_(use_original_pitch)
{
    reset();
}

ImuDelta::~ImuDelta()
{

}


void ImuDelta::updateImu(const sensor_msgs::msg::Imu::SharedPtr &src, sensor_msgs::msg::Imu &res)
{
    updateImu(*src, res);
}

void ImuDelta::updateImu(const sensor_msgs::msg::Imu &src, sensor_msgs::msg::Imu &res)
{
    initialize(src.orientation);

    res.header = src.header;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    am::Rotate::getRPY(src.orientation, roll, pitch, yaw);
    if(use_original_roll_)
    {
        roll = am::Rotate::wrap_pi(roll - init_roll_);
    }
    if(use_original_pitch_)
    {
        pitch = am::Rotate::wrap_pi(pitch - init_pitch_);
    }

    yaw = am::Rotate::wrap_pi(yaw - init_yaw_);

    res = src;
    res.orientation = am::Rotate::toQuaternionMsg(roll, pitch, yaw);
}

void ImuDelta::initialize(const geometry_msgs::msg::Quaternion &q)
{
    if(initialized_)
    {
        return;
    }

    am::Rotate::getRPY(q, init_roll_, init_pitch_, init_yaw_);
    initialized_ = true;
}

void ImuDelta::reset()
{
    init_roll_ = 0.0;
    init_pitch_ = 0.0;
    init_yaw_ = 0.0;
    initialized_ = false;
}

bool ImuDelta::isInitialized() const
{
    return initialized_;
}
}