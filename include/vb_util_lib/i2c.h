/*
 * i2c.h
 *
 *  Created on: Oct. 07, 2017
 *      Author: Automodality
 */

#ifndef VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_I2C_H_
#define VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB_I2C_H_

#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>             // for I2C
#include <linux/i2c-dev.h>      // for I2C
#include <sys/ioctl.h>          // for I2C
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>              // for I2C
#include <stdint.h>

#include <ros/ros.h>

class I2CAdapter
{
public:
  I2CAdapter(int adapter_num);

  void setAdapterNum(int adapter_num) { adapter_num_ = adapter_num; }

protected:
  int adapter_num_; // I2C adapter number. Check back if it is dynamically determined
};

class I2CDevice : public I2CAdapter
{
public:
  I2CDevice(int adatpor_num, int addr);

  int setAddr(int addr);
  double getRateHz();
  int joinI2C(int addr);
  int init();
  int writeI2C(char *buf, int bufsize);
  int readI2C(char *buf, int bufsize);

protected:
  int addr_; // The I2C device address
  double rate_hz_;
  int file_;
};

#endif /* VISBOX_PACKAGES_COMMON_UTILS_UTIL_LIB_INCLUDE_VB_UTIL_LIB__I2C_H_ */
