#ifndef AM_UTILS_IMU_DELTA_CLASS_H_
#define AM_UTILS_IMU_DELTA_CLASS_H_

#include <sensor_msgs/msg/imu.hpp>
#include <vb_util_lib/rotate.h>

namespace am
{
class ImuDelta
{
public:
    ImuDelta(bool use_original_roll, bool use_original_pitch);

    ~ImuDelta();

    void updateImu(const sensor_msgs::msg::Imu::SharedPtr &src, sensor_msgs::msg::Imu &res);

    void updateImu(const sensor_msgs::msg::Imu &src, sensor_msgs::msg::Imu &res);

    void initialize(const geometry_msgs::msg::Quaternion &q);

    void reset();

    bool isInitialized() const;

private:

    bool initialized_ = {false};

    double init_roll_ = {0.0};

    double init_pitch_ = {0.0};

    double init_yaw_ = {0.0};

    bool use_original_roll_ {false};

    bool use_original_pitch_ {false};


};
}

#endif AM_UTILS_IMU_DELTA_CLASS_H_