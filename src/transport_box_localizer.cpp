#include "transport_box_localizer.hpp"

using ceres::Problem;
namespace transport_box_localizer {
TransportBoxLocalizer::TransportBoxLocalizer() : it_since_initialized_(0) {
    ros::NodeHandle nh("~");

    nh.param("detect_up", detect_up_, 2.0);
    nh.param("detect_down", detect_down_, -1.0);
    nh.param("detect_right", detect_right_, 1.0);
    nh.param("detect_left", detect_left_, 1.0);
    nh.param("lidar_intensity", lidar_intensity_, 35000.0);
    nh.param("feature_dist", feature_dist_, 0.15);
    nh.param("initial_x", initial_x_, 0.7);
    nh.param("initial_y", initial_y_, 0.0);
    nh.param("initial_angle", initial_angle_, 0.0);

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
    box_coordinate_pub = nh.advertise<geometry_msgs::PoseStamped>("/taimi/waste_aruco_position", 10);

    // subscribe
    cloudSub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/tim551_cloud", 10, boost::bind(&TransportBoxLocalizer::lidarpointcallback, this, _1));
    startSub = nh.subscribe<std_msgs::String>("/shelf_lifter/cmd", 10, &TransportBoxLocalizer::cmdcallback, this);
    run_behavior_thread_ = new std::thread(std::bind(&TransportBoxLocalizer::runBehavior, this));
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

void TransportBoxLocalizer::estimateBodyPose(vector<box_legs> num_legs) {
    // num legs too little
    if (num_legs.size() < 5) {
        return;
    }
    if (it_since_initialized_ < 1) {
        it_since_initialized_++;
        // processing point
        positions_of_markers_box_leg.resize(positions_of_markers_on_object.size());
        for (int i = 0; i < positions_of_markers_on_object.size(); i++) {
            Eigen::Matrix<double, 4, 1> initialized_box_legs;
            initialized_box_legs(0) = positions_of_markers_on_object(i)(0) * cos(initial_angle_) -
                                      positions_of_markers_on_object(i)(1) * sin(initial_angle_) + initial_x_;
            initialized_box_legs(1) = positions_of_markers_on_object(i)(0) * sin(initial_angle_) +
                                      positions_of_markers_on_object(i)(1) * cos(initial_angle_) + initial_y_;
            initialized_box_legs(2) = positions_of_markers_on_object(i)(2);
            initialized_box_legs(3) = positions_of_markers_on_object(i)(3);

            positions_of_markers_box_leg(i) = initialized_box_legs;
        }
        for (int i = 0; i < positions_of_markers_box_leg.size(); i++) {
            for (int j = 0; j < num_legs.size(); j++) {
                input_data point_legs_input;
                if (sqrt((pow(positions_of_markers_box_leg(i)(0) - num_legs[j].xy_coordinates[0], 2) +
                          pow(positions_of_markers_box_leg(i)(1) - num_legs[j].xy_coordinates[1], 2))) < 0.15) {
                    point_legs_input.a = positions_of_markers_on_object(i)(0);
                    point_legs_input.b = positions_of_markers_on_object(i)(1);
                    point_legs_input.x = num_legs[j].xy_coordinates[0];
                    point_legs_input.y = num_legs[j].xy_coordinates[1];
                    point_legs_correspond.push_back(point_legs_input);
                }
            }
        }

        for (int i = 0; i < point_legs_correspond.size(); i++) {
            ROS_ERROR("a:%f,b:%f,x:%f,y:%f", point_legs_correspond[i].a, point_legs_correspond[i].b,
                      point_legs_correspond[i].x, point_legs_correspond[i].y);
        }

        Problem problem;
        for (int i = 0; i < point_legs_correspond.size(); ++i) {
            CostFunction *cost_function = new AutoDiffCostFunction<F1, 1, 1, 1, 1>(
                new F1(point_legs_correspond[i].x, point_legs_correspond[i].y, point_legs_correspond[i].a,
                       point_legs_correspond[i].b));
            problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), &initial_x_, &initial_y_, &initial_angle_);
        }

        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        std::cout << "Initial x = " << initial_x_ << ", y = " << initial_y_ << ", angle = " << initial_angle_ << "\n";
        Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << summary.FullReport() << "\n";

        iteration_x_ = initial_x_;
        iteration_y_ = initial_y_;
        iteration_angle_ = initial_angle_;
        std::cout << "Final   x: " << initial_x_ << " y: " << initial_y_ << " angle = " << initial_angle_ << "\n";
        geometry_msgs::PoseStamped box_coordinate;

        box_coordinate.header.frame_id = "laser_sick_tim551";
        box_coordinate.header.stamp = ros::Time::now();

        box_coordinate.pose.position.x = initial_x_;
        box_coordinate.pose.position.y = initial_y_;
        tf::Quaternion quat;
        quat = tf::createQuaternionFromYaw(initial_angle_);
        box_coordinate.pose.orientation.w = quat.getW();
        box_coordinate.pose.orientation.x = quat.getX();
        box_coordinate.pose.orientation.y = quat.getY();
        box_coordinate.pose.orientation.z = quat.getZ();

        box_coordinate_pub.publish(box_coordinate);
    } else {
        // processing point
        positions_of_markers_box_leg.resize(positions_of_markers_on_object.size());
        for (int i = 0; i < positions_of_markers_on_object.size(); i++) {
            Eigen::Matrix<double, 4, 1> initialized_box_legs;
            initialized_box_legs(0) = positions_of_markers_on_object(i)(0) * cos(iteration_angle_) -
                                      positions_of_markers_on_object(i)(1) * sin(iteration_angle_) + iteration_x_;
            initialized_box_legs(1) = positions_of_markers_on_object(i)(0) * sin(iteration_angle_) +
                                      positions_of_markers_on_object(i)(1) * cos(iteration_angle_) + iteration_y_;
            initialized_box_legs(2) = positions_of_markers_on_object(i)(2);
            initialized_box_legs(3) = positions_of_markers_on_object(i)(3);

            positions_of_markers_box_leg(i) = initialized_box_legs;
        }

        for (int i = 0; i < positions_of_markers_box_leg.size(); i++) {
            for (int j = 0; j < num_legs.size(); j++) {
                input_data point_legs_input;
                if (sqrt((pow(positions_of_markers_box_leg(i)(0) - num_legs[j].xy_coordinates[0], 2) +
                          pow(positions_of_markers_box_leg(i)(1) - num_legs[j].xy_coordinates[1], 2))) < 0.1) {
                    point_legs_input.a = positions_of_markers_on_object(i)(0);
                    point_legs_input.b = positions_of_markers_on_object(i)(1);
                    point_legs_input.x = num_legs[j].xy_coordinates[0];
                    point_legs_input.y = num_legs[j].xy_coordinates[1];
                    point_legs_correspond.push_back(point_legs_input);
                }
            }
        }
        Problem problem;
        for (int i = 0; i < point_legs_correspond.size(); ++i) {
            CostFunction *cost_function = new AutoDiffCostFunction<F1, 1, 1, 1, 1>(
                new F1(point_legs_correspond[i].x, point_legs_correspond[i].y, point_legs_correspond[i].a,
                       point_legs_correspond[i].b));
            problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), &iteration_x_, &iteration_y_,
                                     &iteration_angle_);
        }
        Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << "Final   x: " << iteration_x_ << " y: " << iteration_y_ << " angle = " << iteration_angle_ << "\n";
        geometry_msgs::PoseStamped box_coordinate;

        box_coordinate.header.frame_id = "laser_sick_tim551";
        box_coordinate.header.stamp = ros::Time::now();

        box_coordinate.pose.position.x = iteration_x_;
        box_coordinate.pose.position.y = iteration_y_;
        tf::Quaternion quat;
        quat = tf::createQuaternionFromYaw(iteration_angle_);
        box_coordinate.pose.orientation.w = quat.getW();
        box_coordinate.pose.orientation.x = quat.getX();
        box_coordinate.pose.orientation.y = quat.getY();
        box_coordinate.pose.orientation.z = quat.getZ();

        box_coordinate_pub.publish(box_coordinate);
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
    ROS_INFO("point_count: %d", point_count);
    laser_filtered_point_pub.publish(filtered_point);

    ROS_INFO("\r\n");

    estimateBodyPose(box_legs_);

    tmp_data.clear();
    box_legs_.clear();
    point_legs_correspond.clear();
    cluster_box_legs_.clear();
    poseArray.poses.clear();
}

void TransportBoxLocalizer::cmdcallback(const std_msgs::String::ConstPtr &msg) {
    if (!msg) {
        return;
    }
    std_msgs::String cmd_str = *msg;

    if (cmd_str.data.find("move_back") != std::string::npos) {
        it_since_initialized_ = 0;
        std::cout << "alignment start!" << std::endl;
    }
}

TransportBoxLocalizer::~TransportBoxLocalizer() {}
} // namespace transport_box_localizer
