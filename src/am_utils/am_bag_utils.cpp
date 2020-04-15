#include <am_utils/am_bag_utils.h>

namespace brain_box {

std::string AMBagUtils::getFileName()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,80,"%Y-%m-%d-%H-%M-%S.bag",timeinfo);
  std::string str(buffer);

  return str;
}

} /* namespace brain_box */
