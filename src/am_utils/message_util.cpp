#include <am_utils/message_util.h>
#include <brain_box_msgs/msg/status.hpp>

std::string MessageUtil::statusIntToString(int status)
{
	 switch(status)
	 {
	 case brain_box_msgs::msg::Status::SUMMARY_GREEN:
		   return "GREEN";
	 case brain_box_msgs::msg::Status::SUMMARY_YELLOW:
		 return "YELLOW";
	 case brain_box_msgs::msg::Status::SUMMARY_RED:
		   return "RED";
	 case brain_box_msgs::msg::Status::SUMMARY_OFFLINE:
		   return "OFFLINE";
	 }
	 return "";
}

std::string MessageUtil::statusIntToChar(int status)
{
	 switch(status)
	 {
	 case brain_box_msgs::msg::Status::SUMMARY_GREEN:
		   return "G";
	 case brain_box_msgs::msg::Status::SUMMARY_YELLOW:
		 return "Y";
	 case brain_box_msgs::msg::Status::SUMMARY_RED:
		   return "R";
	 case brain_box_msgs::msg::Status::SUMMARY_OFFLINE:
		   return "X";
	 }
	 return "";
}
