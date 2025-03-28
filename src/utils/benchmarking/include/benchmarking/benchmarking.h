#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <algorithm>

#include "object_msgs/MultiObstacleBoxMsg.h"

class Benchmarking {
public:
    Benchmarking() = default;

    void initialize(ros::NodeHandle& nh);

    void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
    void rcvTargetCallback(const geometry_msgs::PoseStampedConstPtr& ptr);
    void obstacleCallback(const object_msgs::MultiObstacleBoxMsgConstPtr& obs_msg_ptr);

    void executeCheckCollision(const ros::TimerEvent& event);
    void calculateDynamicObstacleDistance(const ros::TimerEvent& event);
    void calculateObstacleDistance(const ros::TimerEvent& event);
    void displayInfo(const ros::TimerEvent& event);

private:
    ros::Subscriber odom_sub_, obs_sub_, target_sub_;
    ros::Timer      check_timer_, calculate_dynamic_timer_, calculate_timer_, display_timer_;

    bool            reach_target_ {false};
    Eigen::Vector3d target_ {12.0, 0.0, 1.0};

    double start_time_ {0.0}, end_time_ {0.0}, duration_ {0.0};

    double min_dis_ {1e3}, min_dynamic_dis_ {1e3};

    nav_msgs::Odometry               odom_msg_;
    object_msgs::MultiObstacleBoxMsg obs_msg_;
};
