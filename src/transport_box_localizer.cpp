#include "transport_box_localizer.hpp"
namespace transport_box_localizer {
TransportBoxLocalizer::TransportBoxLocalizer() : it_since_initialized_(0) {
    ros::NodeHandle nh("~");
    const std::string pclFilename = nh.param<std::string>("pcd_filename", "");
    mapCloud = loadPointcloudFromPcd(pclFilename);

    nh.param("detect_up", detect_up_, 2.0);
    nh.param("detect_down", detect_down_, -1.0);
    nh.param("detect_right", detect_right_, 1.0);
    nh.param("detect_left", detect_left_, 1.0);
    nh.param("lidar_intensity", lidar_intensity_, 35000.0);
    nh.param("initial_x", initial_x_, 0.7);
    nh.param("initial_y", initial_y_, 0.0);
    nh.param("initial_angle", initial_angle_, 0.0);

    // Create the marker positions from the test points
    List4DPoints positions_of_markers_on_object;
    // Read in the marker positions from the YAML parameter file
    XmlRpc::XmlRpcValue points_list;
    if (!nh.getParam("marker_positions", points_list)) {
        ROS_ERROR(
            "%s: No reference file containing the marker positions, or the "
            "file is improperly formatted. Use the 'marker_positions_file' "
            "parameter in the launch file.",
            ros::this_node::getName().c_str());
        ros::shutdown();
    } else {
        positions_of_markers_on_object.resize(points_list.size());
        for (int i = 0; i < points_list.size(); i++) {
            Eigen::Matrix<double, 4, 1> temp_point;
            temp_point(0) = points_list[i]["x"];
            temp_point(1) = points_list[i]["y"];
            temp_point(2) = points_list[i]["z"];
            temp_point(3) = 1;
            positions_of_markers_on_object(i) = temp_point;
        }
    }

    // publish
    cloudPub = nh.advertise<sensor_msgs::PointCloud2>("icp_map", 10, true);
    laser_filtered_point_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_filtered_point", 10);
    box_legs_array_pub = nh.advertise<geometry_msgs::PoseArray>("box_legs", 10);

    // subscribe
    cloudSub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/tim551_cloud", 10, boost::bind(&TransportBoxLocalizer::lidarpointcallback, this, _1));

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
    ros::Rate rate(10.0);
    while (nh.ok()) {
        // publishCloud(mapCloud, cloudPub, "laser_sick_tim551");
        rate.sleep();
    }
}

void TransportBoxLocalizer::estimateBodyPose(geometry_msgs::PoseArray num_legs) {
    if (it_since_initialized_ < 1) {
        it_since_initialized_++;
    }
}

void TransportBoxLocalizer::lidarpointcallback(const sensor_msgs::PointCloud2::ConstPtr &pointMsgIn) {
    if (pointMsgIn == nullptr) {
        return;
    }
    sensor_msgs::PointCloud2 filtered_point;
    float_union point_x, point_y, intensity;
    vector<uint8_t> tmp_data;
    uint32_t point_count = 0;
    box_legs box_legs_coordinates;
    for (int i = 0; i < pointMsgIn->width; i++) {
        for (int j = 0; j < 4; j++) {
            point_x.cv[j] = pointMsgIn->data[i * 16 + j];
            point_y.cv[j] = pointMsgIn->data[i * 16 + 4 + j];
            intensity.cv[j] = pointMsgIn->data[i * 16 + 12 + j];
        }
        if (point_x.fv > detect_down_ && point_x.fv < detect_up_ && fabs(point_y.fv) < detect_right_ &&
            intensity.fv > lidar_intensity_) {
            point_count++;
            for (int z = 0; z < 16; z++) {
                tmp_data.push_back(pointMsgIn->data[i * 16 + z]);
            }
            box_legs_coordinates.xy_coordinates[0] = point_x.fv;
            box_legs_coordinates.xy_coordinates[1] = point_y.fv;
            box_legs_.push_back(box_legs_coordinates);
        }
    }
    if (box_legs_.size() == 0) {
        box_legs_.clear();
        return;
    }
    // debug
    filtered_point.header = pointMsgIn->header;
    filtered_point.height = pointMsgIn->height;
    filtered_point.width = point_count;
    filtered_point.fields = pointMsgIn->fields;
    filtered_point.is_bigendian = pointMsgIn->is_bigendian;
    filtered_point.point_step = pointMsgIn->point_step;
    filtered_point.row_step = point_count * 16;
    for (int i = 0; i < filtered_point.row_step; i++) {
        filtered_point.data.push_back(tmp_data[i]);
    }
    ROS_DEBUG("point_count: %d", point_count);
    laser_filtered_point_pub.publish(filtered_point);

    float sum_x_pos = 0, sum_y_pos = 0;
    uint32_t classify_number = 0;
    box_legs cluster_box_legs_coordinates;
    bool classify_flag = false;
    for (int i = 0; i < box_legs_.size() - 1; i++) {
        if (pow(pow(box_legs_[i].xy_coordinates[0] - box_legs_[i + 1].xy_coordinates[0], 2) +
                    pow(box_legs_[i].xy_coordinates[1] - box_legs_[i + 1].xy_coordinates[1], 2),
                0.5) < 0.05) {
            sum_x_pos += box_legs_[i].xy_coordinates[0];
            sum_y_pos += box_legs_[i].xy_coordinates[1];
            classify_number++;
            classify_flag = false;
        } else {
            if (classify_number >= 3) {
                cluster_box_legs_coordinates.xy_coordinates[0] = sum_x_pos / classify_number;
                cluster_box_legs_coordinates.xy_coordinates[1] = sum_y_pos / classify_number;

                cluster_box_legs_.push_back(cluster_box_legs_coordinates);

                box_legs_pose.position.x = cluster_box_legs_coordinates.xy_coordinates[0];
                box_legs_pose.position.y = cluster_box_legs_coordinates.xy_coordinates[1];
                box_legs_pose.position.z = 0;
                poseArray.poses.push_back(box_legs_pose);
            }
            classify_number = 0;
            sum_x_pos = 0;
            sum_y_pos = 0;
            classify_flag = true;
        }
    }
    if (!classify_flag && classify_number >= 3) {
        cluster_box_legs_coordinates.xy_coordinates[0] = sum_x_pos / classify_number;
        cluster_box_legs_coordinates.xy_coordinates[1] = sum_y_pos / classify_number;

        cluster_box_legs_.push_back(cluster_box_legs_coordinates);

        box_legs_pose.position.x = cluster_box_legs_coordinates.xy_coordinates[0];
        box_legs_pose.position.y = cluster_box_legs_coordinates.xy_coordinates[1];
        box_legs_pose.position.z = 0;
        poseArray.poses.push_back(box_legs_pose);
    }
    poseArray.header = pointMsgIn->header;
    poseArray.header.stamp = ros::Time::now();
    box_legs_array_pub.publish(poseArray);

    for (int i = 0; i < cluster_box_legs_.size(); i++) {
        ROS_INFO("cluster_box_legs_.xy_coordinates[0]:%f,cluster_box_legs_.box_legs.xy_coordinates[1]:%f",
                 cluster_box_legs_[i].xy_coordinates[0], cluster_box_legs_[i].xy_coordinates[1]);
    }
    ROS_INFO("\r\n");

    box_legs_.clear();
    cluster_box_legs_.clear();
    poseArray.poses.clear();
}

TransportBoxLocalizer::~TransportBoxLocalizer() {}
} // namespace transport_box_localizer
