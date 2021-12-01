#ifndef TRANSPORT_BOX_LOCALIZER_H_
#define TRANSPORT_BOX_LOCALIZER_H_

#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/tf.h>

#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <thread>
#include <vector>

#include "ceres/ceres.h"
#include "geometry_msgs/PoseArray.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "std_msgs/String.h"
using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

using namespace std;
using namespace Eigen;

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

typedef Eigen::Matrix<Eigen::Vector4d, Eigen::Dynamic, 1> List4DPoints;

namespace transport_box_localizer {

typedef union {
    unsigned char cv[4];
    float fv;
} float_union;

typedef struct {
    float xy_coordinates[2];
} box_legs;

typedef struct {
    double x;
    double y;
    double z;
} actual_box_leg;

struct F1 {
    F1(double x, double y, double a, double b) : x_(x), y_(y), a_(a), b_(b) {}
    template <typename T>
    bool operator()(const T *const Tx, const T *const Ty, const T *const Tangle, T *residual) const {
        residual[0] = abs(y_ - Ty[0] - b_ * cos(Tangle[0]) - a_ * sin(Tangle[0])) +
                      abs(x_ - Tx[0] + b_ * sin(Tangle[0]) - a_ * cos(Tangle[0]));
        return true;
    }

private:
    const double x_;
    const double y_;
    const double a_;
    const double b_;
};

class TransportBoxLocalizer {
private:
    Pointcloud::Ptr loadPointcloudFromPcd(const std::string &filename);
    void publishCloud(Pointcloud::Ptr cloud, const ros::Publisher &pub, const std::string &frameId);
    void runBehavior(void);
    void lidarpointcallback(const sensor_msgs::PointCloud2::ConstPtr &pointMsgIn);
    void estimateBodyPose(geometry_msgs::PoseArray num_legs);
    void cmdcallback(const std_msgs::String::ConstPtr &msg);

    Pointcloud::Ptr mapCloud;
    std::thread *run_behavior_thread_;

    ros::Publisher cloudPub;
    ros::Publisher laser_filtered_point_pub;
    ros::Publisher box_legs_array_pub;
    ros::Publisher box_coordinate_pub;
    ros::Subscriber cloudSub;
    ros::Subscriber startSub;

    double detect_up_, detect_down_, detect_right_, detect_left_, lidar_intensity_, feature_dist_;
    double initial_x_, initial_y_, initial_angle_;
    double iteration_x_, iteration_y_, iteration_angle_;

    vector<box_legs> box_legs_;
    vector<box_legs> cluster_box_legs_;

    // Debug
    geometry_msgs::PoseArray poseArray;
    geometry_msgs::Pose box_legs_pose;

    unsigned it_since_initialized_;

    // Create the marker positions from the test points
    List4DPoints positions_of_markers_on_object;
    List4DPoints positions_of_markers_box_leg;
    actual_box_leg positions_of_actual_box_leg[5];

public:
    TransportBoxLocalizer();
    ~TransportBoxLocalizer();
};

} // namespace transport_box_localizer

#endif