#include <am_utils/message_util.h>
#include <brain_box_msgs/Status.h>

std::string MessageUtil::statusIntToString(int status)
{
	 switch(status)
	 {
	 case brain_box_msgs::Status::SUMMARY_GREEN:
		   return "GREEN";
	 case brain_box_msgs::Status::SUMMARY_YELLOW:
		 return "YELLOW";
	 case brain_box_msgs::Status::SUMMARY_RED:
		   return "RED";
	 case brain_box_msgs::Status::SUMMARY_OFFLINE:
		   return "OFFLINE";
	 }
}

std::string MessageUtil::statusIntToChar(int status)
{
	 switch(status)
	 {
	 case brain_box_msgs::Status::SUMMARY_GREEN:
		   return "G";
	 case brain_box_msgs::Status::SUMMARY_YELLOW:
		 return "Y";
	 case brain_box_msgs::Status::SUMMARY_RED:
		   return "R";
	 case brain_box_msgs::Status::SUMMARY_OFFLINE:
		   return "X";
	 }
}
