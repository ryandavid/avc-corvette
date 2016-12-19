#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "motor_control/motor_control_node.h"
#include "motor_control/gpio.h"

#define PI 3.14159265

int fd;

static uint16_t write_to_dac(bool buffered, bool gain, bool shutdown, uint16_t count) {
  // Assembling it bass-ackwards due to clocking out LSB first.
  // TODO: Actually use the other flags.
  uint16_t cmd = ((0x70 + ((count >> 6) & 0x0F)) << 0) + ((count & 0x3F) << 10);

  if (write(fd, &cmd, sizeof(cmd)) != sizeof(cmd)) {
    ROS_ERROR("Write Error!");
  }

}

static void rxJoystickCallback(const sensor_msgs::Joy::ConstPtr& data) {
  static int8_t last_motor_direction = -1;

  // Axes for Xbox controller:
  //  0 : Left Stick Left/Right,    Right is positive
  //  1 : Left Stick Up/Down,       Down is positive
  //  2 : Left Trigger,             Out is -2^15, In is 2^15
  //  3 : Right Trigger,            Out is -2^15, In is 2^15
  //  4 : Right Stick Left/Right,   Right is positive
  //  5 : Right Stick Up/Down,      Down is positive

  // Buttons for Xbox controller:
  //  0 : A
  //  1 : B
  //  2 : X
  //  3 : Y
  //  4 : Left Bumper
  //  5 : Right Bumper

  uint16_t dac_count = ((uint32_t)abs(data->axes[1])) >> 5;

  // Introduce some deadband.
  if(dac_count < 75) {
    dac_count = 0;
  }

  // Make sure we don't overflow.
  if(dac_count > 1023) {
    dac_count = 1023;
  }

  // Note ! since the joystick polarity is opposite polarity than you'd expect.
  bool motor_reverse = !(data->axes[1] < 0);

  // If we swapped the desired direction, flip the GPIOs accordingly.
  if(last_motor_direction != motor_reverse) {
    last_motor_direction = motor_reverse;

    if(motor_reverse) {
      gpio_set_value(kMotorAForwardGPIO, false);
      gpio_set_value(kMotorAReverseGPIO, true);
    } else {
      gpio_set_value(kMotorAForwardGPIO, true);
      gpio_set_value(kMotorAReverseGPIO, false);
    }
  }

  write_to_dac(true, true, true, dac_count);
}


int main(int argc, char **argv) {
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

  if(gpio_export(kMotorAForwardGPIO) || gpio_export(kMotorAReverseGPIO) ||
     gpio_export(kMotorBForwardGPIO) || gpio_export(kMotorBReverseGPIO)) {
    ROS_ERROR("Failed to setup GPIO pins!");
    exit(1);
  }

  // Need to wait some time to let udev perform the chgrp on the newly exported pins.
  ROS_INFO("Exported GPIOs.");
  ros::Duration(0.5).sleep();

  if(gpio_set_dir(kMotorAForwardGPIO, true) || gpio_set_dir(kMotorAReverseGPIO, true) ||
     gpio_set_dir(kMotorBForwardGPIO, true) || gpio_set_dir(kMotorBReverseGPIO, true)) {
    ROS_ERROR("Failed to set direction on GPIO pins!");
    exit(1);
  }

  if(gpio_set_value(kMotorAForwardGPIO, false) || gpio_set_value(kMotorAReverseGPIO, false) ||
     gpio_set_value(kMotorBForwardGPIO, false) || gpio_set_value(kMotorBReverseGPIO, false)) {
    ROS_ERROR("Failed to set value on GPIO pins!");
    exit(1);
  }

  ROS_INFO("Completed setting initial GPIO states.");

  ros::Subscriber joyTopic = node.subscribe<sensor_msgs::Joy>("/joy0", 10, rxJoystickCallback);

  ros::Rate rate(100);

  ROS_INFO("Starting to write to DAC.");
  /*while(ros::ok()) {
    // Generate a sine wave, biased halfway through the DAC count range.
    //count_to_write = (uint32_t)((sin(count * (PI / 180)) * 512.0) + 512.0);
    //dac_cmd = write_to_dac(true, true, true, count_to_write);

    ros::spinOnce();
    rate.sleep();
  }*/

  ros::spin();

  // Make sure we are always setting the output to zero on exit.
  // TODO: Place DAC into Hi-Z mode.
  write_to_dac(true, true, true, 0);

  // Disable all direction control GPIOs.
  gpio_set_value(kMotorAForwardGPIO, false);
  gpio_set_value(kMotorAReverseGPIO, false);
  gpio_set_value(kMotorBForwardGPIO, false);
  gpio_set_value(kMotorBReverseGPIO, false);

  return 0;
} 