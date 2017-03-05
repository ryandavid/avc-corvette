#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <sys/sysctl.h>

#include "ros/ros.h"

#include "system_stats/system_stats.h"
#include "system_stats/SysInfo.h"


int main(int argc, char** argv) {
    // Concat the hostname onto the node name.
    std::string hostname = get_hostname();
    std::string node_name = "system_stats_" + hostname;
    
    // Hyphens are not valid ROS names, so replace with underscores.
    int index = node_name.find("-");
    while (index != std::string::npos) {
        node_name.replace(index, 1, "_");
        index = node_name.find("-", index + 1);
    }

    ros::init(argc, argv, node_name);
    ros::NodeHandle node("~");
    ros::Publisher diag_topic = node.advertise<system_stats::SysInfo>("diag", 10);

    ROS_INFO("Node name: %s", node_name.c_str());

    // No sense copying over the hostname and amount of memory each time.
    system_stats::SysInfo status_msg;
    status_msg.host = hostname;
    status_msg.mem_bytes_total = get_total_mem();;

    ros::Rate rate(1);

    while (ros::ok()) {
        status_msg.header.stamp = ros::Time::now();
        status_msg.mem_bytes_used = get_mem_used();

        get_sys_load(&status_msg.load_avg_1min, &status_msg.load_avg_5min,
            &status_msg.load_avg_15min);

        diag_topic.publish(status_msg);

        ros::spinOnce();
        rate.sleep();
    }
}
