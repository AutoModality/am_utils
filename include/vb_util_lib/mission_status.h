/*
 * mission_status.h
 *
 *  Created on: Nov 23, 2015
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_MISSION_SRC_MISSION_STATUS_H_
#define VISBOX_PACKAGES_MISSION_SRC_MISSION_STATUS_H_

#include <brain_box_msgs/VxMissionStatus.h>
#include <brain_box_msgs/VxFlightPlanStatus.h>

#include <ros/ros.h>

#define DEFAULT_MISSION_STATUS_TOPIC "/ctrl/vx/mission_status"

class MissionStatus {
public:
	MissionStatus() {};
	MissionStatus(ros::NodeHandle *n, std::string topic=DEFAULT_MISSION_STATUS_TOPIC,
	        bool pub_on_change=false, double pub_rate=0.0) :
        topic_(topic), pub_on_change_(pub_on_change), pub_rate_(pub_rate) {
	    initROS(n);
	}

	virtual ~MissionStatus() {};

	void initROS(ros::NodeHandle *n) {
	    if (pub_on_change_ || pub_rate_ > 0.0)
	    {
            publisher_ = true;
            pub_nh_ = n->advertise<brain_box_msgs::VxMissionStatus>(topic_.c_str(), 1000);
	    }
	    else
	    {
            publisher_ = false;
            sub_nh_ = n->subscribe(topic_.c_str(), 1000,
                    &MissionStatus::updateCB, this);
	    }

        if (pub_rate_ > 0)
        {
            pub_timer_ = n->createTimer(ros::Duration(pub_rate_), &MissionStatus::publishTimerCB, this);
            pub_timer_.start();
        }

        ros_init_ = true;
	}

	void setVxMissionStatus(const brain_box_msgs::VxMissionStatus& stat) {
	    setCmdCurrent(stat.cmd_current);
	    setDuration(stat.duration);
	    setError(stat.error);
	    setId(stat.id);
	    setStatus(static_cast<MS_STATUS>(stat.status));
	}

	bool isUpdated() {
		bool ret_val = updated_;
		updated_ = false;
		return ret_val;
	}

	enum class MS_STATUS {IDLE=0, EXECUTING, PAUSED, ABORTED, ERROR_EXIT, DONE};
	const char* STATUS_LABEL[6] {"IDLE", "EXECUTING", "PAUSED", "ABORTED", "ERROR_EXIT", "DONE"};

	brain_box_msgs::VxFlightPlanStatus& getCmdCurrent() {
		return cmd_current_;
	}

	void setCmdCurrent(const brain_box_msgs::VxFlightPlanStatus& cmdCurrent) {
		cmd_current_ = cmdCurrent;
	}

	void updateCmdCurrent(const brain_box_msgs::VxFlightPlanStatus& cmdCurrent) {
		setCmdCurrent(cmdCurrent);
		checkUpdate();
	}

	double getDuration() const {
        return duration_;
	}

    void setDuration(double duration) {
//        ROS_INFO("      SET D = %f", duration);
        duration_ = duration;
    }

    void setDuration() {
        ros::Duration d(ros::Time::now() - getStartTime());
        setDuration(d.toSec());
    }

	void updateDuration() {
		setDuration();
		checkUpdate();
	}

	int getError() const {
		return error_;
	}

	void setError(int error) {
		error_ = error;
	}

	void updateError(int error) {
		setError(error);
		checkUpdate();
	}

	std::string getId() const {
		return id_;
	}

	void setId(std::string id) {
		id_ = id;
	}

	void updateId(std::string id) {
		setId(id);
		checkUpdate();
	}

	MS_STATUS getStatus() const {
		return status_;
	}

	void setStatus(MS_STATUS status) {
		status_ = status;
	}

	void updateStatus(MS_STATUS status) {
		setStatus(status);
		checkUpdate();
	}

	const ros::Time& getStartTime() const {
		return start_time_;
	}

	void setStartTime() {
		start_time_ = ros::Time::now();
		setDuration(0);
	}

    void sendVxMissionStatus() {
        updated_ = true;

        if (!ros_init_ || !publisher_)
        {
            return;
        }
        brain_box_msgs::VxMissionStatus ms;
        ms.cmd_current = getCmdCurrent();
        ms.duration = getDuration();
        ms.error = getError();
        ms.id = getId();
        ms.status = static_cast<unsigned>(getStatus());

        pub_nh_.publish(ms);
    }

    /**
     * Install a callback handler.
     * Use the following sytax to call this function:
     *   setCallback(std::bind(&MyClass::Callback, this, std::placeholders::_1));
     */
    void setCallback(std::function<void(MissionStatus*)> callback)
    {
        callback_ = callback;
    }

    void print() {
        printf("MISSION STATUS\n");
        printf("   Id: %s\n", id_.c_str());
        printf("   topic: %s\n", topic_.c_str());
        printf("   Status: %s\n", STATUS_LABEL[static_cast<int>(status_)]);
        printf("   error: %d\n", error_);
    }

private:
    /**
     * Id of the mission
     */
	std::string id_ {""};

	/**
	 * Mission status
	 */
	MS_STATUS status_;

	/**
	 * Error code, zero if none else error code
	 */
	int error_;

	/**
	 * Current flight plan command being executed
	 */
	brain_box_msgs::VxFlightPlanStatus cmd_current_;

	/**
	 * Time in sec the mission has been running
	 */
	double duration_;

    /**
     * topic to publish updates on
     */
    std::string topic_ {""};

    /**
     * Callback function pointer
     */
    std::function<void(MissionStatus*)> callback_ {NULL};

    /**
     * Check for update condition and publish status message if necessary
     */
    void checkUpdate() {
      //        ROS_INFO("FCU STATUS: checkUpdate IN");
        updated_ = true;
        if (pub_on_change_)
        {
            sendVxMissionStatus();
        }

        if (callback_ != NULL)
        {
            callback_(this);
        }

    //        ROS_INFO("FCU STATUS: checkUpdate OUT");
    }

	/**
	 * Start time of the mission
	 */
	ros::Time start_time_;

	/**
	 * Updated flag
	 */
	bool updated_ {false};

    /**
     * Publish on update flag
     */
    bool pub_on_change_ = false;

    /**
     * Rate to publish status if not 0
     */
    double pub_rate_ = 0.0;

    /**
     * True if this instance is publishing versus consuming the data
     */
    bool publisher_ {false};

    /**
	 * ROS handles for publishing and receiving status
	 */
	bool ros_init_ {false};
	ros::Publisher pub_nh_;
	ros::Subscriber sub_nh_;
    ros::Timer pub_timer_;

    /**
     * Timer callback for publishing stats messages
     */
    void publishTimerCB(const ros::TimerEvent& te) {
        sendVxMissionStatus();
    }

	void updateCB(const brain_box_msgs::VxMissionStatus::ConstPtr& msg) {
        updated_ = true;
        setVxMissionStatus(*msg);

        if (callback_ != NULL)
        {
            callback_(this);
        }
	}
};

#endif /* VISBOX_PACKAGES_MISSION_SRC_MISSION_STATUS_H_ */
