#ifndef TRANSPORT_BOX_LOCALIZER_H_
#define TRANSPORT_BOX_LOCALIZER_H_

#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>

#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <thread>

#include "geometry_msgs/PoseArray.h"
#include "pointmatcher/PointMatcher.h"
using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints;

namespace transport_box_localizer {

typedef union {
    unsigned char cv[4];
    float fv;
} float_union;

typedef struct {
    float xy_coordinates[2];
} box_legs;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class TransportBoxLocalizer {
private:
    Pointcloud::Ptr loadPointcloudFromPcd(const std::string &filename);
    void publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId);
    void runBehavior(void);
    void lidarpointcallback(const sensor_msgs::PointCloud2::ConstPtr &pointMsgIn);
    DP fromPCL(const Pointcloud &pcl);

    Pointcloud::Ptr mapCloud;
    std::thread *run_behavior_thread_;

    ros::Publisher cloudPub;
    ros::Publisher laser_filtered_point_pub;
    ros::Publisher box_legs_array_pub;
    ros::Subscriber cloudSub;

    double detect_up_, detect_down_, detect_right_, detect_left_, lidar_intensity_;

    vector<box_legs> box_legs_;
    vector<box_legs> cluster_box_legs_;

    // Debug
    geometry_msgs::PoseArray poseArray;
    geometry_msgs::Pose box_legs_pose;

public:
    TransportBoxLocalizer();
    ~TransportBoxLocalizer();
};

} // namespace transport_box_localizer

#endif