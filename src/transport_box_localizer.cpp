#include "transport_box_localizer.hpp"
namespace transport_box_localizer {
TransportBoxLocalizer::TransportBoxLocalizer() {
    ros::NodeHandle nh("~");
    const std::string pclFilename = nh.param<std::string>("pcd_filename", "");
    mapCloud = loadPointcloudFromPcd(pclFilename);
    // publish
    cloudPub = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 1, true);
    // subscribe
    cloudSub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/tim551_cloud", 100, boost::bind(&TransportBoxLocalizer::laserpointcallback, this, _1));

    // Create the default ICP algorithm
    PM::ICP icp;
    // See the implementation of setDefault() to create a custom ICP algorithm
    icp.setDefault();
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
    ros::Rate rate(15.0);
    while (nh.ok()) {
        publishCloud(mapCloud, cloudPub, "world");
        rate.sleep();
    }
}

void TransportBoxLocalizer::laserpointcallback(const sensor_msgs::PointCloud2::ConstPtr &pointMsgIn) {
    if (pointMsgIn == nullptr) {
        return;
    }
}

TransportBoxLocalizer::~TransportBoxLocalizer() {}
} // namespace transport_box_localizer
