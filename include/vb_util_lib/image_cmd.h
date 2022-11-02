/*
 * image_cmd.h
 *
 *  Created on: Feb 25, 2016
 *      Author: ubuntu
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMAGE_CMD_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMAGE_CMD_H_

#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <vb_util_lib/json.h>
#include <vb_util_lib/task_trigger.h>
#include <vb_util_lib/fcu_control.h>

enum class IMAGE_CMD {
    VIDEO=0,
    SNAPSHOT=1,
    PERIODIC_STILLS=2,
    UNKNOWN=3,
};

extern std::map<IMAGE_CMD, std::string> imageCmd_2_string;
extern std::map<std::string, IMAGE_CMD> string_2_imageCmd;

#define IMAGECMD_TO_STRING(m) imageCmd_2_string[m]
#define STRING_TO_IMAGECMD(p) string_2_imageCmd[p]
//#define PX4_TO_MODE_MSG(p) (static_cast<unsigned int>(px4_2_mode[p]))


class ImageCmd {
public:
    ImageCmd();
    virtual ~ImageCmd();

    bool loadJSON(Json::Value& fc);

    IMAGE_CMD getCmd() const {
        return cmd_;
    }

    const TaskTrigger& getEndTrigger() const {
        return end_trigger_;
    }

    double getPeriod() const {
        return period_;
    }

    const TaskTrigger& getStartTrigger() const {
        return start_trigger_;
    }

    bool executeCmd();

    void setUav(FCUControl* uav) {
        uav_ = uav;
    }

private:
    /**
     * FCU interface for specific command
     */
    FCUControl* uav_ {NULL};

    /**
     * This is the nature of the image command
     */
    IMAGE_CMD cmd_ {IMAGE_CMD::UNKNOWN};

    /**
     * If taking pictures this determines the period of the pictures.
     * If <= 0 then it is a single snap shot
     */
    double period_ {0.0};

    /**
     * Start trigger for image task
     */
    TaskTrigger start_trigger_;

    /**
     * End trigger for image task
     */
    TaskTrigger end_trigger_;
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_IMAGE_CMD_H_ */
