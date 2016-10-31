#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "serial/serial.h"
#include <tf/transform_datatypes.h>

#include "ahrs400_driver/ahrs400_driver.h"

const int kApproxPublishRateHz = 60;
const int kDiagnosticPublishRateHz = 1;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ahrs400_driver_node");
  ros::NodeHandle node;

  std::string inertial_topic_name;
  std::string magnetic_topic_name;
  std::string serial_port_name;
  int serial_port_baudrate;

  inertial_topic_name = ros::this_node::getName() + "/inertial";
  magnetic_topic_name = ros::this_node::getName() + "/magnetic";
  node.param("serial_port_name", serial_port_name, std::string("/dev/ttyUSB0"));
  node.param("serial_port_baudrate", serial_port_baudrate, 38400);

  ROS_INFO("Node '%s'.", ros::this_node::getName().c_str());
  ROS_INFO("Publishing measurements to '%s' and '%s'.",
           inertial_topic_name.c_str(),
           magnetic_topic_name.c_str());
  ROS_INFO("Using serial port '%s' at %d baud.", serial_port_name.c_str(), serial_port_baudrate);

  ros::Publisher inertialTopic = node.advertise<sensor_msgs::Imu>(inertial_topic_name, 100);
  ros::Publisher magneticTopic = node.advertise<sensor_msgs::MagneticField>(magnetic_topic_name, 100);
  ros::Publisher diagnosticTopic = node.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 10);

  xbow_ahrs400 ahrs(serial_port_name, serial_port_baudrate);
  ROS_INFO("AHRS part number: %s", ahrs.getVersion());
  ROS_INFO("AHRS serial number: %d", ahrs.getSerialNumber());

  ahrs.setMode(ANGLE_MODE, CONTINUOUS_MODE);

  ROS_INFO("Starting to publish measurements!");

  uint32_t cycleCount = 0;
  uint32_t numErrors = 0;
  while (ros::ok() == true) {
    // Publish a sensor measurement.
    if(ahrs.getMeasurement() == false) {
      numErrors++;
      ROS_ERROR("Failed to get measurement.");
    } else {
      sensor_msgs::Imu inertialMsg;
      inertialMsg.header.stamp = ros::Time::now();

      tf::Quaternion q;
      q.setEuler(ahrs.measurements.attitude[2],   // Yaw, angle about Z.
                 ahrs.measurements.attitude[1],   // Pitch, angle about Y.
                 ahrs.measurements.attitude[0]);  // Roll, angle about X.
      inertialMsg.orientation.x = q.x();
      inertialMsg.orientation.y = q.y();
      inertialMsg.orientation.z = q.z();
      inertialMsg.orientation.w = q.w();

      inertialMsg.angular_velocity.x = ahrs.measurements.angularRate[0];
      inertialMsg.angular_velocity.y = ahrs.measurements.angularRate[1];
      inertialMsg.angular_velocity.z = ahrs.measurements.angularRate[2];

      inertialMsg.linear_acceleration.x = ahrs.measurements.accel[0];
      inertialMsg.linear_acceleration.y = ahrs.measurements.accel[1];
      inertialMsg.linear_acceleration.z = ahrs.measurements.accel[2];

      inertialTopic.publish(inertialMsg);

      sensor_msgs::MagneticField magneticMsg;
      magneticMsg.header.stamp = ros::Time::now();
      magneticMsg.magnetic_field.x = ahrs.measurements.magnetometer[0];
      magneticMsg.magnetic_field.y = ahrs.measurements.magnetometer[1];
      magneticMsg.magnetic_field.z = ahrs.measurements.magnetometer[2];
      magneticTopic.publish(magneticMsg);
    }

    // Publish diagnostic message approximately once a second.
    if(cycleCount > (kApproxPublishRateHz / kDiagnosticPublishRateHz)) {
      diagnostic_msgs::DiagnosticStatus msg;
      diagnostic_msgs::KeyValue keyval;
      std::stringstream tempString;

      msg.name = ros::this_node::getName();

      tempString << ahrs.getVersion() << ", " << ahrs.getSerialNumber();
      msg.hardware_id = tempString.str();

      if(numErrors > 0) {
        msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        msg.message = "Failed to read measurement.";
      } else {
        msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        msg.message = "OK.";
      }

      keyval.key = "temperature";
      tempString.str("");
      tempString << ahrs.measurements.temperature;
      keyval.value = tempString.str();
      msg.values.push_back(keyval);

      keyval.key = "readErrors";
      tempString.str("");
      tempString << numErrors;
      keyval.value = tempString.str();
      msg.values.push_back(keyval);

      diagnosticTopic.publish(msg);

      cycleCount = 0;
      numErrors = 0;
    } else {
      cycleCount++;
    }

    ros::spinOnce();
  }

  return 0;
}

