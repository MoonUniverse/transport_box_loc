#include <ros/ros.h>

#include "transport_box_localizer.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "localizer_node");

    ros::NodeHandle n;

    transport_box_localizer::TransportBoxLocalizer TransportBoxLocalizer_;

    ros::spin();

    return (0);
}
