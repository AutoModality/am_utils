/*
 * image_cmd.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: ubuntu
 */

#include <vb_util_lib/image_cmd.h>

std::map<IMAGE_CMD, std::string> imageCmd_2_string = {
        {IMAGE_CMD::VIDEO, "VIDEO"},
        {IMAGE_CMD::SNAPSHOT, "SNAPSHOT"},
        {IMAGE_CMD::PERIODIC_STILLS, "PERIODIC_STILLS"},
        {IMAGE_CMD::UNKNOWN, "UNKNOWN"},
};

std::map<std::string, IMAGE_CMD> string_2_imageCmd = {
        {"VIDEO", IMAGE_CMD::VIDEO},
        {"SNAPSHOT", IMAGE_CMD::SNAPSHOT},
        {"PERIODIC_STILLS", IMAGE_CMD::PERIODIC_STILLS},
        {"UNKNOWN", IMAGE_CMD::UNKNOWN},
};

ImageCmd::ImageCmd() {
    // TODO Auto-generated constructor stub

}

ImageCmd::~ImageCmd() {
    // TODO Auto-generated destructor stub
}

bool ImageCmd::loadJSON(Json::Value& fc) {
    std::string p = fc.get("cmd", "").asString();
    if (!(p == "VIDEO" ||
            p == "SNAPSHOT" ||
            p == "PERIODIC_STILLS"))
    {
        ROS_ERROR("MS: Missing or invalid image command in mission spec");
        return false;
    }
    cmd_ = STRING_TO_IMAGECMD(p);
    period_ = fc.get("period", 0.0).asDouble();
    ROS_INFO("    Loading image command %s [period:%0.2f]", p.c_str(), period_);

    Json::Value tr = fc["Start_trigger"];
    if (tr.isObject())
    {
        if (!start_trigger_.loadJSON(tr))
        {
            return false;
        }
    }
    tr = fc["End_trigger"];
    if (tr.isObject())
    {
       if (!end_trigger_.loadJSON(tr))
        {
            return false;
        }
    }

    return true;
}

bool ImageCmd::executeCmd() {
}
