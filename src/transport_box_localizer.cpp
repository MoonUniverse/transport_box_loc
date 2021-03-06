#include "transport_box_localizer.hpp"

namespace transport_box_localizer {
TransportBoxLocalizer::TransportBoxLocalizer() : it_since_initialized_(0) {
    ros::NodeHandle nh("~");

    nh.param("detect_up", detect_up_, 2.0);
    nh.param("detect_down", detect_down_, -1.0);
    nh.param("detect_right", detect_right_, 1.0);
    nh.param("detect_left", detect_left_, 1.0);
    nh.param("lidar_intensity", lidar_intensity_, 30000.0);
    nh.param("feature_dist", feature_dist_, 0.15);
    nh.param("initial_x", initial_x_, 0.7);
    nh.param("initial_y", initial_y_, 0.0);
    nh.param("initial_angle", initial_angle_, 0.0);
    nh.param("base_sick_link", base_sick_link_, 0.223);

    nh.param("inflation_coefficient", inflation_coefficient_, 0.005);
    nh.param("inflation_number", inflation_number_, 5);

    nh.param("error_tolerance", error_tolerance_, 0.05);

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

    length_ = fabs(positions_of_markers_on_object(0)(0) - positions_of_markers_on_object(1)(0));
    width_ = fabs(positions_of_markers_on_object(4)(1) - positions_of_markers_on_object(0)(1));

    // publish
    laser_filtered_point_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_filtered_point", 10);
    box_legs_array_pub = nh.advertise<geometry_msgs::PoseArray>("box_legs", 10);
    box_coordinate_pub = nh.advertise<geometry_msgs::PoseStamped>("box_coordinate", 10);

    // subscribe
    cloudSub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/tim551_cloud", 10, boost::bind(&TransportBoxLocalizer::lidarpointcallback, this, _1));
    startSub = nh.subscribe<std_msgs::String>("/shelf_lifter/cmd", 10, &TransportBoxLocalizer::cmdcallback, this);
    run_behavior_thread_ = new std::thread(std::bind(&TransportBoxLocalizer::runBehavior, this));
}

void TransportBoxLocalizer::runBehavior(void) {
    ros::NodeHandle nh;
    ros::Rate rate(1.0);
    while (nh.ok()) {
        rate.sleep();
    }
}

void TransportBoxLocalizer::estimateInitalPose(vector<filter_cloudpoint> cloudpointMsg) {
    double sum_x_pos = 0, sum_y_pos = 0;
    uint32_t classify_number = 0;
    bool classify_flag = false;
    cluster_cloudpoint temp_cluster_cloudpoint_;
    // Clustering
    for (int i = 0; i < cloudpointMsg.size() - 1; i++) {
        if (pow(pow(cloudpointMsg[i].x - cloudpointMsg[i + 1].x, 2) +
                    pow(cloudpointMsg[i].y - cloudpointMsg[i + 1].y, 2),
                0.5) < 0.05) {
            sum_x_pos += cloudpointMsg[i].x;
            sum_y_pos += cloudpointMsg[i].y;
            classify_number++;
            classify_flag = false;
        } else {
            if (classify_number >= 5) {
                temp_cluster_cloudpoint_.x = sum_x_pos / classify_number;
                temp_cluster_cloudpoint_.y = sum_y_pos / classify_number;

                cluster_cloudpoint_.push_back(temp_cluster_cloudpoint_);
            }
            classify_number = 0;
            sum_x_pos = 0;
            sum_y_pos = 0;
            classify_flag = true;
        }
    }
    if (!classify_flag && classify_number >= 5) {
        temp_cluster_cloudpoint_.x = sum_x_pos / classify_number;
        temp_cluster_cloudpoint_.y = sum_y_pos / classify_number;

        cluster_cloudpoint_.push_back(temp_cluster_cloudpoint_);
        classify_number = 0;
        sum_x_pos = 0;
        sum_y_pos = 0;
    }

    if (cluster_cloudpoint_.size() < 2) {
        return;
        ROS_ERROR("No much point!");
    }
    float feature_distance = 5.0;
    for (int i = 0; i < cluster_cloudpoint_.size() - 1; i++) {
        for (int j = i + 1; j < cluster_cloudpoint_.size(); j++) {
            double point_distance = hypot((cluster_cloudpoint_[i].x - cluster_cloudpoint_[j].x),
                                          (cluster_cloudpoint_[i].y - cluster_cloudpoint_[j].y));
            if (fabs(point_distance - feature_dist_) < 0.05) {
                // find circles
                double expect_x = 0, expect_y = 0;
                double x1 = cluster_cloudpoint_[i].x;
                double y1 = cluster_cloudpoint_[i].y;
                double x2 = cluster_cloudpoint_[j].x;
                double y2 = cluster_cloudpoint_[j].y;
                // circles R
                double R = hypot(feature_dist_ / 2, width_ / 2);

                double c1 = 0, c2 = 0, A = 0, B = 0, C = 0, y01 = 0, x01 = 0, x02 = 0, y02 = 0;
                if (x1 != x2) {
                    c1 = (pow(x2, 2) - pow(x1, 2) + pow(y2, 2) - pow(y1, 2)) / 2 / (x2 - x1);
                    c2 = (y2 - y1) / (x2 - x1);
                    A = 1.0 + pow(c2, 2);
                    B = 2 * (x1 - c1) * c2 - 2 * y1;
                    C = pow((x1 - c1), 2) + pow(y1, 2) - pow(R, 2);
                    y01 = (-B + sqrt(B * B - 4 * A * C)) / 2 / A;
                    x01 = c1 - c2 * y01;
                    y02 = (-B - sqrt(B * B - 4 * A * C)) / 2 / A;
                    x02 = c1 - c2 * y02;
                    if (y02 < y01) {
                        expect_x = x02;
                        expect_y = y02;
                    } else {
                        expect_x = x01;
                        expect_y = y01;
                    }
                    float distance = hypot(expect_x, expect_y);
                    if (distance < feature_distance) {
                        feature_distance = distance;
                        if (hypot(cluster_cloudpoint_[i].x, cluster_cloudpoint_[i].y) <
                            hypot(cluster_cloudpoint_[j].x, cluster_cloudpoint_[j].y)) {
                            initial_point_[0].x = cluster_cloudpoint_[i].x;
                            initial_point_[0].y = cluster_cloudpoint_[i].y;
                            initial_point_[1].x = cluster_cloudpoint_[j].x;
                            initial_point_[1].y = cluster_cloudpoint_[j].y;
                        } else {
                            initial_point_[0].x = cluster_cloudpoint_[j].x;
                            initial_point_[0].y = cluster_cloudpoint_[j].y;
                            initial_point_[1].x = cluster_cloudpoint_[i].x;
                            initial_point_[1].y = cluster_cloudpoint_[i].y;
                        }
                    }
                } else {
                    ROS_ERROR("find circle center fail");
                }
            }
        }
    }
    double error_angle =
        atan2((initial_point_[1].y - initial_point_[0].y), (initial_point_[1].x - initial_point_[0].x));
    for (int i = 0; i < cluster_cloudpoint_.size() - 1; i++) {
        double current_distance =
            hypot(cluster_cloudpoint_[i].x - initial_point_[0].x, cluster_cloudpoint_[i].y - initial_point_[0].y);
        double current_angle =
            atan2((cluster_cloudpoint_[i].y - initial_point_[0].y), (cluster_cloudpoint_[i].x - initial_point_[0].x));
        if (fabs(current_distance - length_) < 0.05) {
            if (fabs(current_angle - error_angle) < 0.05) {
                initial_point_[2].x = cluster_cloudpoint_[i].x;
                initial_point_[2].y = cluster_cloudpoint_[i].y;
            }
        } else if (fabs(current_distance - width_) < 0.05) {
            if (fabs(fabs(current_angle - error_angle) - 1.57) < 0.05) {
                initial_point_[3].x = cluster_cloudpoint_[i].x;
                initial_point_[3].y = cluster_cloudpoint_[i].y;
            }
        }
    }
    for (int i = 0; i < 4; i++) {
        debugPose.position.x = initial_point_[i].x;
        debugPose.position.y = initial_point_[i].y;
        debugPose.position.z = 0;
        poseArray.poses.push_back(debugPose);
    }
    // debug init pose
    poseArray.header.frame_id = "base_link";
    poseArray.header.stamp = ros::Time::now();
    box_legs_array_pub.publish(poseArray);

    cluster_cloudpoint_.clear();
}

void TransportBoxLocalizer::estimateBodyPose(vector<filter_cloudpoint> cloudpointMsg) {
    // num legs too little
    if (cloudpointMsg.size() < 15) {
        return;
    }

    if (it_since_initialized_ < 1) {
        it_since_initialized_++;

        estimateInitalPose(cloudpointMsg);

        ceres_input point_legs_input;
        for (int i = 0; i < 3; i++) {
            point_legs_input.a = positions_of_markers_on_object(4 - i)(0);
            point_legs_input.b = positions_of_markers_on_object(4 - i)(1);
            point_legs_input.x = initial_point_[i].x;
            point_legs_input.y = initial_point_[i].y;
            point_legs_correspond_.push_back(point_legs_input);
        }
        point_legs_input.a = positions_of_markers_on_object(0)(0);
        point_legs_input.b = positions_of_markers_on_object(0)(1);
        point_legs_input.x = initial_point_[3].x;
        point_legs_input.y = initial_point_[3].y;
        point_legs_correspond_.push_back(point_legs_input);

        Problem problem;
        for (int i = 0; i < point_legs_correspond_.size(); ++i) {
            CostFunction *cost_function = new AutoDiffCostFunction<F1, 1, 1, 1, 1>(
                new F1(point_legs_correspond_[i].x, point_legs_correspond_[i].y, point_legs_correspond_[i].a,
                       point_legs_correspond_[i].b));
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

        box_coordinate.header.frame_id = "base_link";
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
            for (int j = 0; j < cloudpointMsg.size(); j++) {
                ceres_input point_legs_input;
                if (sqrt((pow(positions_of_markers_box_leg(i)(0) - cloudpointMsg[j].x, 2) +
                          pow(positions_of_markers_box_leg(i)(1) - cloudpointMsg[j].y, 2))) < 0.1) {
                    point_legs_input.a = positions_of_markers_on_object(i)(0);
                    point_legs_input.b = positions_of_markers_on_object(i)(1);
                    point_legs_input.x = cloudpointMsg[j].x;
                    point_legs_input.y = cloudpointMsg[j].y;
                    point_legs_correspond_.push_back(point_legs_input);
                }
            }
        }
        Problem problem;
        for (int i = 0; i < point_legs_correspond_.size(); ++i) {
            CostFunction *cost_function = new AutoDiffCostFunction<F1, 1, 1, 1, 1>(
                new F1(point_legs_correspond_[i].x, point_legs_correspond_[i].y, point_legs_correspond_[i].a,
                       point_legs_correspond_[i].b));
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

        box_coordinate.header.frame_id = "base_link";
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
    float_union inflation_point_x, inflation_point_y, inflation_point_z, inflation_intensity;
    vector<uint8_t> tmp_data;
    filter_cloudpoint filter_cloudpoint_coordinates;

    for (int i = 0; i < pointMsgIn->width; i++) {
        for (int j = 0; j < 4; j++) {
            point_x.cv[j] = pointMsgIn->data[i * 16 + j];
            point_y.cv[j] = pointMsgIn->data[i * 16 + 4 + j];
            intensity.cv[j] = pointMsgIn->data[i * 16 + 12 + j];
        }

        if (point_x.fv > detect_down_ && point_x.fv < detect_up_ && fabs(point_y.fv) < detect_right_ &&
            intensity.fv > lidar_intensity_) {
            point_x.fv = point_x.fv + base_sick_link_;
            double length = hypot(point_x.fv, point_y.fv);
            for (int i = 0; i < inflation_number_; i++) {
                filter_cloudpoint_coordinates.x = sin(atan2(point_x.fv, point_y.fv)) *
                                                  ((i - (inflation_number_ - 1) / 2) * inflation_coefficient_ + length);
                filter_cloudpoint_coordinates.y = cos(atan2(point_x.fv, point_y.fv)) *
                                                  ((i - (inflation_number_ - 1) / 2) * inflation_coefficient_ + length);
                filter_cloudpoint_.push_back(filter_cloudpoint_coordinates);

                inflation_point_x.fv = filter_cloudpoint_coordinates.x;
                inflation_point_y.fv = filter_cloudpoint_coordinates.y;
                inflation_point_z.fv = 0;
                inflation_intensity.fv = intensity.fv;

                for (int j = 0; j < 4; j++) {
                    tmp_data.push_back(inflation_point_x.cv[j]);
                }
                for (int j = 0; j < 4; j++) {
                    tmp_data.push_back(inflation_point_y.cv[j]);
                }
                for (int j = 0; j < 4; j++) {
                    tmp_data.push_back(inflation_point_z.cv[j]);
                }
                for (int j = 0; j < 4; j++) {
                    tmp_data.push_back(inflation_intensity.cv[j]);
                }
            }
        }
    }

    if (filter_cloudpoint_.size() == 0) {
        filter_cloudpoint_.clear();
        return;
    }
    // debug
    filtered_point.header.frame_id = "base_link";
    filtered_point.header.stamp = ros::Time::now();
    filtered_point.height = pointMsgIn->height;
    filtered_point.width = filter_cloudpoint_.size();
    filtered_point.fields = pointMsgIn->fields;
    filtered_point.is_bigendian = pointMsgIn->is_bigendian;
    filtered_point.point_step = pointMsgIn->point_step;
    filtered_point.row_step = filter_cloudpoint_.size() * 16;
    for (int i = 0; i < filtered_point.row_step; i++) {
        filtered_point.data.push_back(tmp_data[i]);
    }
    laser_filtered_point_pub.publish(filtered_point);

    estimateBodyPose(filter_cloudpoint_);

    tmp_data.clear();
    filter_cloudpoint_.clear();
    point_legs_correspond_.clear();
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
