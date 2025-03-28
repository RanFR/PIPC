#ifndef VIRTUAL_MAP_H
#define VIRTUAL_MAP_H

#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>

#include "object_msgs/MultiObstacleBoxMsg.h"
#include "object_msgs/ObstaclePoints.h"

enum class ObstacleStatus { ForwardAcc, ReverseAcc, ForwardDec, ReverseDec };

class VirtualMap {
public:
    VirtualMap() = default;

    void initialize(ros::NodeHandle& nh);

    void generateFixedWall();
    void generateComplexObstacles();

    void generateSixMovingCylinders();
    void generateSixConstSpeedCylinders();

    void calculateObstaclePosition(ObstacleStatus& status,
                                   double&         vel,
                                   const double    max_vel,
                                   const double    acc,
                                   const double    dt);
    void calculateConstObstaclePosition(double& vel, double cur_pos, double min_pos, double max_pos);

    void publishVirtualMapTimer(const ros::TimerEvent& event);

private:
    ros::Publisher virtual_map_pub_, multi_obstacle_boxes_pub_;
    ros::Timer     virtual_map_timer_;

    pcl::PointCloud<pcl::PointXYZ> cloud_map_;
    sensor_msgs::PointCloud2       virtual_map_;

    /* Obstacles */
    enum MovingState { Deceleration, Forward, Reverse };
    ObstacleStatus status1_ {ObstacleStatus::ForwardAcc};
    ObstacleStatus status2_ {ObstacleStatus::ForwardAcc};
    ObstacleStatus status3_ {ObstacleStatus::ForwardAcc};
    ObstacleStatus status4_ {ObstacleStatus::ForwardAcc};
    ObstacleStatus status5_ {ObstacleStatus::ForwardAcc};
    ObstacleStatus status6_ {ObstacleStatus::ForwardAcc};

    Eigen::Vector3d cylinder1_pos_, cylinder2_pos_, cylinder3_pos_, cylinder4_pos_, cylinder5_pos_, cylinder6_pos_;

    /* Basic virtual map parameters */
    double      resolution_;
    double      update_rate_;
    std::string map_type_;
    std::string frame_id_ {std::string("world")};

    // Fixed wall parameters
    double wall_thickness_;
    double gap_width_;
};

#endif
