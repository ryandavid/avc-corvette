#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "serial/serial.h"
#include "std_msgs/ByteMultiArray.h"

static void rxNtripCallback(const std_msgs::ByteMultiArray::ConstPtr& data) {
	ROS_INFO("Recieved %lu bytes!", data->data.size());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "bd950_driver_node");
  ros::NodeHandle node("~");

  std::string serial_port_name;
  int serial_port_baudrate;

  node.param("port", serial_port_name, std::string("/dev/ttyUSB0"));
  node.param("baudrate", serial_port_baudrate, 115200);

  std::string ntrip_topic_name;
  node.param("ntrip_topic", ntrip_topic_name, std::string("/ntrip/data"));

  ROS_INFO("Node '%s'.", ros::this_node::getName().c_str());
  ROS_INFO("Using serial port '%s' at %d baud.", serial_port_name.c_str(), serial_port_baudrate);


  ros::Publisher navTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 10);
  ros::Subscriber rtcmTopic = node.subscribe<std_msgs::ByteMultiArray>(ntrip_topic_name, 10, rxNtripCallback);

  ros::Rate rate(50);

  while(ros::ok()) {
  	ros::spinOnce();
  	rate.sleep();
  }

  return -1;
}