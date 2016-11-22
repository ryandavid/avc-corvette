#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>

#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/KeyValue.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include "std_msgs/ByteMultiArray.h"
#include "bd950_driver/NavSVInfoArray.h"
#include "bd950_driver/NavDebug.h"

#include "bd950_driver/bd950_driver.h"


std::vector<uint8_t> ntripData;

// Since this will only be called when 'spinOnce' is called, so we don't have to
// worry about a race between this and when it gets written to the serial port.
static void rxNtripCallback(const std_msgs::ByteMultiArray::ConstPtr& data) {
	ROS_DEBUG("Recieved %lu bytes from NTRIP!", data->data.size());

	ntripData.insert(ntripData.end(), data->data.begin(), data->data.end());
}

// http://www.trimble.com/OEM_ReceiverHelp/v4.85/en/GSOFmessages_GSOF.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "bd950_driver_node");
  ros::NodeHandle node("~");

  std::string serial_port_name, ntrip_topic_name;
  int serial_port_baudrate;

  node.param("port", serial_port_name, std::string("/dev/ttyUSB0"));
  node.param("baudrate", serial_port_baudrate, 115200);
  node.param("ntrip_topic", ntrip_topic_name, std::string("/ntrip/data"));

  ROS_INFO("Node '%s'.", ros::this_node::getName().c_str());
  ROS_INFO("Using serial port '%s' at %d baud.", serial_port_name.c_str(), serial_port_baudrate);

  ros::Publisher navTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 10);
  ros::Publisher velTopic = node.advertise<geometry_msgs::TwistStamped>("vel", 10);
  ros::Publisher svTopic = node.advertise<bd950_driver::NavSVInfoArray>("svInfo", 10);
  ros::Publisher debugTopic = node.advertise<bd950_driver::NavDebug>("debug", 10);
  ros::Subscriber rtcmTopic = node.subscribe<std_msgs::ByteMultiArray>(ntrip_topic_name, 10, rxNtripCallback);

  ros::Rate rate(50);

  bd950 gps(serial_port_name, serial_port_baudrate);
  
  while(ros::ok()) {
  	if(gps.process() == true) {
      // Position.
      if(gps.rxMsgs[bd950::LLH] == true) {
        sensor_msgs::NavSatFix navMsg;
        sensor_msgs::NavSatStatus navStatus;

        bool hasFix = gps.positionTimeUTC.isDifferential;
        bool isSBAS = gps.positionTimeUTC.isDifferential;
        bool isRTK = gps.positionTimeUTC.isNetworkRTK | gps.positionTimeUTC.isLocationRTK;

        if(hasFix & isRTK) {
          navStatus.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        } else if(hasFix & isSBAS) {
          navStatus.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        } else if(hasFix) {
          navStatus.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        } else {
          navStatus.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        navStatus.service = sensor_msgs::NavSatStatus::SERVICE_GPS; 

        navMsg.header.stamp = ros::Time::now();
        navMsg.status = navStatus;
        navMsg.latitude = gps.llh.latitude;
        navMsg.longitude = gps.llh.longitude;
        navMsg.altitude = gps.llh.height;

        navMsg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
        navMsg.position_covariance[0] = gps.varianceCovariance.xx;
        navMsg.position_covariance[1] = gps.varianceCovariance.xy;
        navMsg.position_covariance[2] = gps.varianceCovariance.xz;

        navMsg.position_covariance[3] = gps.varianceCovariance.xy;
        navMsg.position_covariance[4] = gps.varianceCovariance.yy;
        navMsg.position_covariance[5] = gps.varianceCovariance.yz;

        navMsg.position_covariance[6] = gps.varianceCovariance.xz;
        navMsg.position_covariance[7] = gps.varianceCovariance.yz;
        navMsg.position_covariance[8] = gps.varianceCovariance.zz;

        navTopic.publish(navMsg);
        gps.rxMsgs[bd950::LLH] = false;
      }

      // Debug.
      if(gps.rxMsgs[bd950::PositionTimeUTC] == true) {
        bd950_driver::NavDebug debugMsg;

        debugMsg.header.stamp = ros::Time::now();

        debugMsg.numOfSV = gps.positionTimeUTC.numOfSV;
        debugMsg.newPosition = gps.positionTimeUTC.newPosition;
        debugMsg.newClockThisSolution = gps.positionTimeUTC.newClockThisSolution;
        debugMsg.newHorizontalThisSolution = gps.positionTimeUTC.newHorizontalThisSolution;
        debugMsg.newHeightThisSolution = gps.positionTimeUTC.newHeightThisSolution;
        debugMsg.usesLeastSquaresPosition = gps.positionTimeUTC.usesLeastSquaresPosition;
        debugMsg.usesFilteredL1 = gps.positionTimeUTC.usesFilteredL1;
        debugMsg.isDifferential = gps.positionTimeUTC.isDifferential;
        debugMsg.isPhase = gps.positionTimeUTC.isPhase;
        debugMsg.isFixedInteger = gps.positionTimeUTC.isFixedInteger;
        debugMsg.isOmnistar = gps.positionTimeUTC.isOmnistar;
        debugMsg.isStatic = gps.positionTimeUTC.isStatic;
        debugMsg.isNetworkRTK = gps.positionTimeUTC.isNetworkRTK;
        debugMsg.isLocationRTK = gps.positionTimeUTC.isLocationRTK;
        debugMsg.isBeaconDGPS = gps.positionTimeUTC.isBeaconDGPS;
        debugMsg.initCounter = gps.positionTimeUTC.initCounter;

        debugTopic.publish(debugMsg);
        gps.rxMsgs[bd950::PositionTimeUTC] = false;
      }

      // Velocity.
      if(gps.rxMsgs[bd950::Velocity] == true) {
        geometry_msgs::TwistStamped twistMsg;
        twistMsg.header.stamp = ros::Time::now();
        twistMsg.twist.linear.x = gps.velocity.horizontalVelocity * sin(gps.velocity.heading);
        twistMsg.twist.linear.y = gps.velocity.horizontalVelocity * cos(gps.velocity.heading);
        twistMsg.twist.linear.z = gps.velocity.verticalVelocity;
        velTopic.publish(twistMsg);
        gps.rxMsgs[bd950::Velocity] = false;
      }

      // SV.
      if(gps.rxMsgs[bd950::SVDetail] == true) {
    		bd950_driver::NavSVInfoArray svMsg;
    		svMsg.header.stamp = ros::Time::now();

    		svMsg.sv.resize(gps.svInfo.size());

    		for(size_t i = 0; i < gps.svInfo.size(); i++) {
    			svMsg.sv[i].prn = gps.svInfo[i].prn;
    			svMsg.sv[i].trackedL1 = gps.svInfo[i].trackedL1;
    			svMsg.sv[i].trackedL2 = gps.svInfo[i].trackedL2;
    			svMsg.sv[i].trackedAtBaseL1 = gps.svInfo[i].trackedAtBaseL1;
    			svMsg.sv[i].trackedAtBaseL2 = gps.svInfo[i].trackedAtBaseL2;
    			svMsg.sv[i].usedInSln = gps.svInfo[i].usedInSolution;
    			svMsg.sv[i].usedInRTK = gps.svInfo[i].usedInRTK;
    			svMsg.sv[i].trackedPcodeL1 = gps.svInfo[i].trackedPcodeL1;
    			svMsg.sv[i].trackedPcodeL2 = gps.svInfo[i].trackedPcodeL2;
    			svMsg.sv[i].elevation = gps.svInfo[i].elevation;
    			svMsg.sv[i].azimuth = gps.svInfo[i].azimuth;
    			svMsg.sv[i].snrL1 = gps.svInfo[i].snrL1;
    			svMsg.sv[i].snrL2 = gps.svInfo[i].snrL2;
    		}
    		svTopic.publish(svMsg);
        gps.rxMsgs[bd950::SVDetail] = false;
      }
    }

    // Write out any correction data received from NTRIP.
  	if(ntripData.size() > 0) {
  		gps.writeCorrectionData(ntripData); 
  		ntripData.clear();
  	}

  	ros::spinOnce();
  	rate.sleep();
  }

  return 0;
}
