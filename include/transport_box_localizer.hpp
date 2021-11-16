#ifndef TRANSPORT_BOX_LOCALIZER_H_
#define TRANSPORT_BOX_LOCALIZER_H_

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>

#include <thread>

#include "pointmatcher/PointMatcher.h"

using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

namespace transport_box_localizer {
class TransportBoxLocalizer {
private:
    Pointcloud::Ptr loadPointcloudFromPcd(const std::string &filename);
    void publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId);
    void runBehavior(void);
    Pointcloud::Ptr mapCloud;
    std::thread *run_behavior_thread_;

    ros::Publisher cloudPub;

public:
    TransportBoxLocalizer();
    ~TransportBoxLocalizer();
};

} // namespace transport_box_localizer

#endif