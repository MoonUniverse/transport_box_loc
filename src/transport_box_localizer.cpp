#include "transport_box_localizer.hpp"
namespace transport_box_localizer {
TransportBoxLocalizer::TransportBoxLocalizer() {
    ros::NodeHandle nh("~");
    cloudPub = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 1, true);
    const std::string pclFilename = nh.param<std::string>("pcd_filename", "");
    mapCloud = loadPointcloudFromPcd(pclFilename);
    run_behavior_thread_ = new std::thread(std::bind(&TransportBoxLocalizer::runBehavior, this));
}

Pointcloud::Ptr TransportBoxLocalizer::loadPointcloudFromPcd(const std::string &filename) {
    Pointcloud::Ptr cloud(new Pointcloud);
    pcl::PCLPointCloud2 cloudBlob;
    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    return cloud;
}

void TransportBoxLocalizer::publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId) {
    cloud->header.frame_id = frameId;
    cloud->header.seq = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
}

void TransportBoxLocalizer::runBehavior(void) {
    ros::NodeHandle nh;
    ros::Rate rate(25.0);
    while (nh.ok()) {
        publishCloud(mapCloud, cloudPub, "cloud_frame");
        rate.sleep();
    }
}

TransportBoxLocalizer::~TransportBoxLocalizer() {}
} // namespace transport_box_localizer
