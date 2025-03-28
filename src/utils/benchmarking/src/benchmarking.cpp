#include "benchmarking/benchmarking.h"

void Benchmarking::initialize(ros::NodeHandle& nh) {
    std::string odometry_topic, obstacle_topic, target_topic;
    nh.param<std::string>("odometry_topic", odometry_topic, std::string("/odometry"));
    nh.param<std::string>("obstacle_topic", obstacle_topic, std::string("/obstacle"));
    nh.param<std::string>("target_topic", target_topic, std::string("/move_base_simple/goal"));

    odom_sub_   = nh.subscribe<nav_msgs::Odometry>(odometry_topic, 1, &Benchmarking::odometryCallback, this);
    target_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(target_topic, 1, &Benchmarking::rcvTargetCallback, this);
    obs_sub_ = nh.subscribe<object_msgs::MultiObstacleBoxMsg>(obstacle_topic, 1, &Benchmarking::obstacleCallback, this);

    check_timer_ = nh.createTimer(ros::Duration(1e-3), &Benchmarking::executeCheckCollision, this);
    calculate_dynamic_timer_ =
        nh.createTimer(ros::Duration(1e-3), &Benchmarking::calculateDynamicObstacleDistance, this);
    calculate_timer_ = nh.createTimer(ros::Duration(1e-3), &Benchmarking::calculateObstacleDistance, this);
    display_timer_   = nh.createTimer(ros::Duration(1.0), &Benchmarking::displayInfo, this);
}

void Benchmarking::odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
    odom_msg_ = *odom_msg_ptr;

    Eigen::Vector3d current_pos {odom_msg_ptr->pose.pose.position.x, odom_msg_ptr->pose.pose.position.y, 1.0};
    if ((current_pos - target_).norm() <= 2.0 && !reach_target_) {
        end_time_     = ros::Time::now().toSec();
        duration_     = end_time_ - start_time_;
        reach_target_ = true;
    }
}

void Benchmarking::rcvTargetCallback(const geometry_msgs::PoseStampedConstPtr& ptr) {
    start_time_ = ros::Time::now().toSec();
}

void Benchmarking::obstacleCallback(const object_msgs::MultiObstacleBoxMsgConstPtr& obs_msg_ptr) {
    obs_msg_ = *obs_msg_ptr;
}

void Benchmarking::executeCheckCollision(const ros::TimerEvent& event) {
    if (std::abs(odom_msg_.pose.pose.position.x - 6.0) < 0.5 && std::abs(odom_msg_.pose.pose.position.y - 0.5) < 0.5) {
        ROS_ERROR("Hit obstacle!");
        return;
    } else {
        double dis {1e3};
        double x {odom_msg_.pose.pose.position.x}, y {odom_msg_.pose.pose.position.y};

        auto norm_res = [](double a, double b) -> double {
            return std::sqrt(std::pow(a, 2) + std::pow(b, 2));
        };

        if (x <= 5.5 && y <= 0.0) {
            dis = norm_res(x - 5.5, y);
        } else if (x <= 5.5 && y <= 1.0) {
            dis = std::abs(x - 5.5);
        } else if (x <= 5.5) {
            dis = norm_res(x - 5.5, y - 1.0);
        } else if (x <= 6.5 && y <= 0.0) {
            dis = std::abs(y);
        } else if (x <= 6.5) {
            dis = std::abs(y - 1.0);
        } else if (y <= 0.0) {
            dis = norm_res(x - 6.5, y);
        } else if (y <= 1.0) {
            dis = std::abs(x - 6.5);
        } else {
            dis = norm_res(x - 6.5, y - 1.0);
        }

        if (dis < 0.1) {
            ROS_ERROR("Hit obstacle!");
            return;
        }
    }

    for (int i = 0; i < obs_msg_.box.size(); ++i) {
        double temp_x {odom_msg_.pose.pose.position.x - obs_msg_.box[i].x};
        double temp_y {odom_msg_.pose.pose.position.y - obs_msg_.box[i].y};
        if (temp_x * temp_x + temp_y * temp_y < 0.36) {
            ROS_ERROR("Hit obstacle!");
            return;
        }
    }
}

void Benchmarking::calculateDynamicObstacleDistance(const ros::TimerEvent& event) {
    for (int i = 0; i < obs_msg_.box.size(); ++i) {
        double temp_x {odom_msg_.pose.pose.position.x - obs_msg_.box[i].x};
        double temp_y {odom_msg_.pose.pose.position.y - obs_msg_.box[i].y};
        double min_dynamic_dis {std::sqrt(temp_x * temp_x + temp_y * temp_y)};
        if (min_dynamic_dis < min_dynamic_dis_) {
            min_dynamic_dis_ = min_dynamic_dis;
        }
    }
}

void Benchmarking::calculateObstacleDistance(const ros::TimerEvent& event) {
    double static_dis_x {std::abs(odom_msg_.pose.pose.position.x - 6.0)};
    double static_dis_y {std::abs(odom_msg_.pose.pose.position.y)};
    double min_static_dis = std::sqrt(static_dis_x * static_dis_x + static_dis_y * static_dis_y);
    if (min_static_dis < min_dis_) {
        min_dis_ = min_static_dis;
    }

    for (int i = 0; i < obs_msg_.box.size(); ++i) {
        double temp_x {odom_msg_.pose.pose.position.x - obs_msg_.box[i].x};
        double temp_y {odom_msg_.pose.pose.position.y - obs_msg_.box[i].y};
        double min_dynamic_dis {std::sqrt(temp_x * temp_x + temp_y * temp_y)};
        if (min_dynamic_dis < min_dis_) {
            min_dis_ = min_dynamic_dis;
        }
    }
}

void Benchmarking::displayInfo(const ros::TimerEvent& event) {
    if (!reach_target_) {
        ROS_INFO("Min dy dis: %f, min dis: %f", min_dynamic_dis_, min_dis_);
    } else {
        ROS_INFO("Time: %f, min dy dis: %f, min dis: %f", duration_, min_dynamic_dis_, min_dis_);
    }
}
