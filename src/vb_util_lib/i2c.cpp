/*
 * i2c.cpp
 *
 *  Created on: Oct. 07, 2017
 *      Author: AutoModality
 */

#include <vb_util_lib/i2c.h>

I2CAdapter::I2CAdapter(int adapter_num) :
  adapter_num_(adapter_num)
{
}

I2CDevice::I2CDevice(rclcpp::Node::SharedPtr node, int adapter_num, int addr) :
  I2CAdapter(adapter_num), addr_(addr), node_(node)
{
	node_->get_parameter_or("rate_hz", rate_hz_, 10.0);
}

int I2CDevice::setAddr(int addr)
{
  addr_ = addr;

  return 0;
}

double I2CDevice::getRateHz()
{
  return rate_hz_;
}

int I2CDevice::joinI2C(int addr)
{
  // Check if the I2C device is accessible through its address
  if (ioctl(file_, I2C_SLAVE, addr) < 0)
  {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    perror("Failed to acquire bus access and/or talk to slave.");
    return -1;
  }

  return 1;
}

int I2CDevice::init()
{
  char filename[20];
  char buff[10];

  //  Each registered i2c adapter gets a number, counting from 0. You can
  //  examine /sys/class/i2c-dev/ to see what number corresponds to which adapter.
  //  Alternatively, you can run "i2cdetect -l" to obtain a formatted list of all
  //  i2c adapters present on your system at a given time. Adapter numbers are
  //  assigned somewhat dynamically, so you can not assume much about them.
  //  They can even change from one boot to the next. Do the check in a loop
  //  to check which one is for the I2C device with certain address.

  //for (int i = 0; i <= 8; i++)
  for (int i = adapter_num_; i <= 8; i++)
  {
    snprintf(filename, 19, "/dev/i2c-%d", i);
    file_ = open(filename, O_RDWR);
    RCLCPP_INFO(node_->get_logger(), "I2C device: %s", filename);

    // Check if the I2C adapter is accessible
    if (file_ < 0)
    {
      /* ERROR HANDLING; you can check errno to see what went wrong */
      perror("Failed to open the i2c bus");
      continue;
    }

    // Join I2C device with default address
    if (joinI2C(addr_) < 0)
      continue;

    // This part seems not required. Should be removed.
    // Command 'o' is to fade all lights out. Use this command to
    // check if BlinkM is on this I2C adapter
    //if (turnOff() < 0)
    //  continue;

    setAdapterNum(i);
    RCLCPP_INFO_STREAM(node_->get_logger(), "I2C device with address " << addr_ << " is on I2C-" << i);

    return 1;
  }

  return -1;
}

int I2CDevice::writeI2C(char *buf, int bufsize)
{
  if (write(file_, buf, bufsize) != bufsize)
  {
    /* ERROR HANDLING: i2c transaction failed */
    perror("Failed to write to the i2c bus.");
    return -1;
  }

  return 1;
}

int I2CDevice::readI2C(char *buf, int bufsize)
{
  if (read(file_, buf, bufsize) != bufsize)
  {
    /* ERROR HANDLING: i2c transaction failed */
    perror("Failed to read the i2c bus.");
    return -1;
  }

  return 1;
}
