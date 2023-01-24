#include <iostream>
#include <fstream>

#include <vb_util_lib/dji_challenge_util.h>
#include <vb_util_lib/json_utils.h>

brain_box_msgs::msg::VisionCmnd DJIChallengeUtil::selectMode(Mode mode)
{
	Json::Value root;
	Json::Reader reader;
	std::string file_name;
	if(mode == Mode::SURVIVOR)
	{
		file_name = "marker_config_survivor.json";
	}
	else
	{
		file_name = "marker_config_landing.json";
	}
	std::ifstream file("/home/ubuntu/catkin_ws/src/visbox/packages/vision/vision/launch/" + file_name);
	file >> root;
	brain_box_msgs::msg::VisionCmnd vision_cmnd;
	AMJSONUtils::jsonToCtrlVisionCmnd(root, vision_cmnd);
	return vision_cmnd;
}



