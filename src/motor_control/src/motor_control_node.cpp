#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>

#include "ros/ros.h"

#include "motor_control/motor_control_node.h"

#define PI 3.14159265

static uint16_t generate_dac_cmd(bool buffered, bool gain, bool shutdown, uint16_t count) {
  // Assembling it bass-ackwards due to clocking out LSB first.
  // TODO: Actually use the other flags.
  return ((0x70 + ((count >> 6) & 0x0F)) << 0) + ((count & 0x3F) << 10);
}


int main(int argc, char **argv) {
  int fd;
  uint32_t count = 0;
  uint32_t count_to_write;
  uint16_t dac_cmd;

  ros::init(argc, argv, "motor_controller_node");
  ros::NodeHandle node("~");

  fd = open("/dev/spidev1.0", O_RDWR);
  if (fd <= 0) {
    ROS_ERROR("Could not open SPI port.");
    exit(1);
  }

  ros::Rate rate(500);

  ROS_INFO("Starting to write to DAC.");
  while(ros::ok()) {
    // Generate a sine wave, biased halfway through the DAC count range.
    count_to_write = (uint32_t)((sin(count * (PI / 180)) * 512.0) + 512.0);
    dac_cmd = generate_dac_cmd(true, true, true, count_to_write);

    if (write(fd, &dac_cmd, sizeof(dac_cmd)) != sizeof(dac_cmd)) {
      ROS_ERROR("Write Error!");
    }

    count++;

    ros::spinOnce();
    rate.sleep();
  }

  // Make sure we are always setting the output to zero on exit.
  // TODO: Place DAC into Hi-Z mode.
  dac_cmd = generate_dac_cmd(true, true, true, 0);
  if (write(fd, &dac_cmd, sizeof(dac_cmd)) != sizeof(dac_cmd)) {
    ROS_ERROR("Write Error!");
  }

  return 0;
} 