#include <vb_util_lib/time_util.h>

std::string TimeUtil::printTime(boost::posix_time::ptime timestamp)
{
	std::string etime = to_iso_extended_string(timestamp);
	std::size_t dotPos = etime.find('.');
	if(dotPos == std::string::npos)
	{
		return etime + ".000Z";
	}
	else
	{
		std::string prefix = etime.substr(0, dotPos + 1);
		long microSecs = atol(etime.substr(dotPos + 1).c_str());
		char milliSecs[4];
		if(microSecs < 1000000)
		{
			sprintf(milliSecs, "%.3ld", (long)(((float)microSecs / 1000.0) + 0.5));
		}
		else
		{
			// something is wrong - not in microseconds
			sprintf(milliSecs, "000");
		}
		return prefix + milliSecs + 'Z';
	}
}
