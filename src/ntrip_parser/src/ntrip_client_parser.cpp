#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>

#include "ros/ros.h"
#include "ntrip_client/ByteArray.h"

#include "ntrip_parser/RTCM.h"
#include "ntrip_parser/DecodedNtrip.h"

RTCM *decoder;
float header_stamp_secs;

static void rxNtripCallback(const ntrip_client::ByteArray::ConstPtr& data) {
    ROS_DEBUG("Received %lu bytes", data->data.size());

    header_stamp_secs = data->header.stamp.sec;

    for(size_t i = 0; i < data->data.size(); i++) {
        decoder->new_byte(data->data[i]);
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ntrip_parser_node");
  ros::NodeHandle node("~");

  ros::Publisher decodedTopic = node.advertise<ntrip_parser::DecodedNtrip>("decoded", 10);
  ros::Subscriber ntripTopic = node.subscribe<ntrip_client::ByteArray>("/ntrip/data", 10, rxNtripCallback);

  decoder = new RTCM(0, true);

  ros::Rate rate(1.0);
  while(ros::ok()) {
    ROS_INFO("Last Observation: %f", decoder->last_rx_observation);
    ROS_INFO("Msg Header: %d", (int)header_stamp_secs % 3600);

    ntrip_parser::DecodedNtrip decodedMsg;

    decodedMsg.header.stamp = ros::Time::now();

    decodedMsg.station_id = decoder->station_id;
    decodedMsg.station_health = decoder->health;
    decodedMsg.last_rx_observation = decoder->last_rx_observation;
    decodedMsg.ref_x = decoder->ref_x;
    decodedMsg.ref_y = decoder->ref_y;
    decodedMsg.ref_z = decoder->ref_z;

    decodedMsg.rx_msgs.clear();
    std::map<uint32_t, uint32_t>::iterator i;
    for(i = decoder->rx_msg_counts.begin(); i != decoder->rx_msg_counts.end(); i++) {
        ntrip_parser::KeyInt rx_msg_count;
        std::stringstream messageType;
        messageType << i->first;

        rx_msg_count.key = messageType.str();
        rx_msg_count.value = i->second;
        decodedMsg.rx_msgs.push_back(rx_msg_count);
    }
    decodedTopic.publish(decodedMsg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
} 