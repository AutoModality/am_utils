#ifndef TARGETENUKINEMATICS_H_
#define TARGETENUKINEMATICS_H_

#include <vb_util_lib/kinematics.h>
#include <nav_msgs/Odometry.h>
#include <vb_util_lib/topics.h>
// #include <vb_util_lib/bag_logger.h>  // TODO (dan) bag_logger

class TargetENUKinematics: public Kinematics {
public:
    TargetENUKinematics();
    TargetENUKinematics(ros::NodeHandle *n, std::string topic=am::am_topics::CTRL_VX_TARGET_CURRENT_ENU,
            bool pub_on_change=false, double pub_rate=0.0) :
                Kinematics(n, topic!="" ? topic : am::am_topics::CTRL_VX_TARGET_CURRENT_ENU,
                		Asset_ENU, pub_on_change, pub_rate) {
    }
    virtual ~TargetENUKinematics() {};

private:
};

#endif /* TARGETENUKINEMATICS_H_ */
