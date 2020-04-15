#ifndef TARGETENUKINEMATICS_H_
#define TARGETENUKINEMATICS_H_

#include <vb_util_lib/kinematics.h>
#include <nav_msgs/Odometry.h>
// #include <vb_util_lib/bag_logger.h>  // TODO (dan) bag_logger

#define TARGET_CURRENT_ENU_TOPIC "/ctrl/vx/target/currentENU"

class TargetENUKinematics: public Kinematics {
public:
    TargetENUKinematics();
    TargetENUKinematics(ros::NodeHandle *n, std::string topic=TARGET_CURRENT_ENU_TOPIC,
            bool pub_on_change=false, double pub_rate=0.0) :
                Kinematics(n, topic!="" ? topic : TARGET_CURRENT_ENU_TOPIC,
                		Asset_ENU, pub_on_change, pub_rate) {
    }
    virtual ~TargetENUKinematics() {};

private:
};

#endif /* TARGETENUKINEMATICS_H_ */
