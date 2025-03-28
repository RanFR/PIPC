#include "virtual_map/virtual_map.h"

void VirtualMap::initialize(ros::NodeHandle& nh) {
    // Virtual map basic parameters
    nh.param("resolution", resolution_, 0.1);
    nh.param("map_type", map_type_, std::string("fixed_wall"));
    nh.param("frame_id", frame_id_, std::string("world"));
    nh.param("update_rate", update_rate_, 10.0);

    // Fixed wall parameters
    nh.param("fixed_wall/gap", gap_width_, 1.0);
    nh.param("fixed_wall/thickness", wall_thickness_, 0.2);

    // Obstacle position
    double init_x, init_y, init_z;
    nh.param("obstacle/init_x", init_x, 5.0);
    nh.param("obstacle/init_y", init_y, 0.0);
    nh.param("obstacle/init_z", init_z, 0.0);

    // Publishers, to publish virtual map and center pose of the virtual cylinders
    virtual_map_pub_          = nh.advertise<sensor_msgs::PointCloud2>("pcd", 1);
    multi_obstacle_boxes_pub_ = nh.advertise<object_msgs::MultiObstacleBoxMsg>("obstacle_box", 1);

    virtual_map_timer_ = nh.createTimer(ros::Duration(1.0 / update_rate_), &VirtualMap::publishVirtualMapTimer, this);

    if (map_type_ == std::string("complex")) {
        status1_ = ObstacleStatus::ReverseAcc;
        status2_ = ObstacleStatus::ForwardDec;

        cylinder1_pos_ = Eigen::Vector3d(4.0, 4.5, 0.0);
        cylinder2_pos_ = Eigen::Vector3d(8.0, 0.0, 0.0);
    }
    if (map_type_ == std::string("six")) {
        status1_       = ObstacleStatus::ForwardAcc;
        status3_       = ObstacleStatus::ForwardAcc;
        status5_       = ObstacleStatus::ForwardAcc;
        cylinder1_pos_ = Eigen::Vector3d(5.0, -4.0, 0.0);
        cylinder3_pos_ = Eigen::Vector3d(15.0, -4.0, 0.0);
        cylinder5_pos_ = Eigen::Vector3d(25.0, -4.0, 0.0);

        status2_       = ObstacleStatus::ReverseAcc;
        status4_       = ObstacleStatus::ReverseAcc;
        status6_       = ObstacleStatus::ReverseAcc;
        cylinder2_pos_ = Eigen::Vector3d(10.0, 4.0, 0.0);
        cylinder4_pos_ = Eigen::Vector3d(20.0, 4.0, 0.0);
        cylinder6_pos_ = Eigen::Vector3d(30.0, 4.0, 0.0);
    }
    if (map_type_ == std::string("six_const")) {
        cylinder1_pos_ = Eigen::Vector3d(4.0, -3.0, 0.0);
        cylinder3_pos_ = Eigen::Vector3d(12.0, -3.0, 0.0);
        cylinder5_pos_ = Eigen::Vector3d(20.0, -3.0, 0.0);

        cylinder2_pos_ = Eigen::Vector3d(8.0, 3.0, 0.0);
        cylinder4_pos_ = Eigen::Vector3d(16.0, 3.0, 0.0);
        cylinder6_pos_ = Eigen::Vector3d(24.0, 3.0, 0.0);
    }
}

void VirtualMap::generateFixedWall() {
    // 清空点云
    cloud_map_.points.clear();

    // 距离正前方5m
    Eigen::Vector3d gap_position(5.0, -1.0, 0.0);
    // y轴长度20m，z轴高度5m
    Eigen::Vector3d wall_size(wall_thickness_, 20.0, 2.5);

    for (double lx = gap_position.x() - wall_size.x() / 2.0; lx <= gap_position.x() + wall_size.x() / 2.0;
         lx += resolution_) {
        for (double ly = -wall_size.y() / 2; ly < wall_size.y() / 2 + resolution_; ly += resolution_) {
            for (double lz = 0.0; lz < wall_size.z(); lz += resolution_) {
                if (std::abs(ly - gap_position.y()) <= gap_width_ / 2.0) {
                    continue;
                }

                pcl::PointXYZ pt;
                pt.x = lx;
                pt.y = ly;
                pt.z = lz;
                cloud_map_.points.push_back(pt);
            }
        }
    }

    cloud_map_.width    = cloud_map_.points.size();
    cloud_map_.height   = 1;
    cloud_map_.is_dense = true;
}

/**
 * @brief Generate a complex obstacle map. 生成一个复杂动静障碍地图
 */
void VirtualMap::generateComplexObstacles() {
    // Clear cloud map vector
    cloud_map_.clear();

    /* 静态障碍物 */
    for (double dx = -0.5; dx <= 0.5; dx += resolution_) {
        for (double dy = -0.5; dy <= 0.5; dy += resolution_) {
            for (double z = 0.0; z <= 5.0; z += resolution_) {
                pcl::PointXYZ pt(6.0 + dx, 0.5 + dy, z);
                cloud_map_.push_back(pt);
            }
        }
    }

    /* 动态障碍物 */
    const double  dt = 1 / update_rate_;
    static double vel1 {0.0}, vel2 {2.0};

    cylinder1_pos_.y() += vel1 * dt;
    cylinder2_pos_.y() += vel2 * dt;

    calculateObstaclePosition(status1_, vel1, 3.0, 1.0, dt);
    calculateObstaclePosition(status2_, vel2, 2.0, 0.5, dt);

    // Face to the cylinder, the cylinder is 1m in radius, 5m in z direction
    Eigen::Vector3d cuboid_size(1.0, 1.0, 5.0);

    for (double lx = -cuboid_size.x() / 2; lx < cuboid_size.x() / 2 + resolution_; lx += resolution_) {
        for (double ly = -cuboid_size.y() / 2; ly < cuboid_size.y() / 2 + resolution_; ly += resolution_) {
            for (double lz = 0.0; lz < cuboid_size.z() + resolution_; lz += resolution_) {
                // In the cylinder
                if (std::pow(lx, 2) + std::pow(ly, 2) < std::pow(0.5, 2)) {
                    Eigen::Vector3d obs_body(lx, ly, lz);
                    Eigen::Vector3d obs_world1, obs_world2;
                    obs_world1 = obs_body + cylinder1_pos_;
                    obs_world2 = obs_body + cylinder2_pos_;

                    pcl::PointXYZ pt;
                    pt.x = obs_world1.x();
                    pt.y = obs_world1.y();
                    pt.z = obs_world1.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world2.x();
                    pt.y = obs_world2.y();
                    pt.z = obs_world2.z();
                    cloud_map_.push_back(pt);
                }
            }
        }
    }

    pcl::toROSMsg(cloud_map_, virtual_map_);

    // MODIFIED: 发布动态障碍物位置
    object_msgs::MultiObstacleBoxMsg msg;

    // 移动障碍物1
    object_msgs::ObstacleBoxMsg msg1;
    msg1.x = cylinder1_pos_.x(), msg1.y = cylinder1_pos_.y(), msg1.z = 2.5;  // 中心点
    msg1.x_width = 1.0, msg1.y_width = 1.0, msg1.z_width = 5.0;              // 宽度

    // 移动障碍物2
    object_msgs::ObstacleBoxMsg msg2;
    msg2.x = cylinder2_pos_.x(), msg2.y = cylinder2_pos_.y(), msg2.z = 2.5;  // 中心点
    msg2.x_width = 1.0, msg2.y_width = 1.0, msg2.z_width = 5.0;              // 宽度

    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.box.push_back(msg1);
    msg.box.push_back(msg2);
    multi_obstacle_boxes_pub_.publish(msg);
}

/**
 * @brief 生成包含6个动态障碍物的地图
 */
void VirtualMap::generateSixMovingCylinders() {
    // Clear cloud map vector
    cloud_map_.clear();

    /* 动态障碍物 */
    const double  dt      = 1.0 / update_rate_;
    const double  max_vel = 2.0;
    const double  acc     = 0.5;
    static double vel1 {0.0}, vel2 {0.0}, vel3 {0.0}, vel4 {0.0}, vel5 {0.0}, vel6 {0.0};

    cylinder1_pos_.y() += vel1 * dt;
    cylinder2_pos_.y() += vel2 * dt;
    cylinder3_pos_.y() += vel3 * dt;
    cylinder4_pos_.y() += vel4 * dt;
    cylinder5_pos_.y() += vel5 * dt;
    cylinder6_pos_.y() += vel6 * dt;

    calculateObstaclePosition(status1_, vel1, max_vel, acc, dt);
    calculateObstaclePosition(status2_, vel2, max_vel, acc, dt);
    calculateObstaclePosition(status3_, vel3, max_vel, acc, dt);
    calculateObstaclePosition(status4_, vel4, max_vel, acc, dt);
    calculateObstaclePosition(status5_, vel5, max_vel, acc, dt);
    calculateObstaclePosition(status6_, vel6, max_vel, acc, dt);

    // Face to the cylinder, the cylinder is 1m in radius, 5m in z direction
    Eigen::Vector3d cuboid_size(1.0, 1.0, 5.0);

    for (double lx = -cuboid_size.x() / 2; lx < cuboid_size.x() / 2 + resolution_; lx += resolution_) {
        for (double ly = -cuboid_size.y() / 2; ly < cuboid_size.y() / 2 + resolution_; ly += resolution_) {
            for (double lz = 0.0; lz < cuboid_size.z() + resolution_; lz += resolution_) {
                // In the cylinder
                if (std::pow(lx, 2) + std::pow(ly, 2) < std::pow(0.5, 2)) {
                    Eigen::Vector3d obs_body(lx, ly, lz);
                    Eigen::Vector3d obs_world1, obs_world2, obs_world3, obs_world4, obs_world5, obs_world6;
                    obs_world1 = obs_body + cylinder1_pos_;
                    obs_world2 = obs_body + cylinder2_pos_;
                    obs_world3 = obs_body + cylinder3_pos_;
                    obs_world4 = obs_body + cylinder4_pos_;
                    obs_world5 = obs_body + cylinder5_pos_;
                    obs_world6 = obs_body + cylinder6_pos_;

                    pcl::PointXYZ pt;
                    pt.x = obs_world1.x();
                    pt.y = obs_world1.y();
                    pt.z = obs_world1.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world2.x();
                    pt.y = obs_world2.y();
                    pt.z = obs_world2.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world3.x();
                    pt.y = obs_world3.y();
                    pt.z = obs_world3.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world4.x();
                    pt.y = obs_world4.y();
                    pt.z = obs_world4.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world5.x();
                    pt.y = obs_world5.y();
                    pt.z = obs_world5.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world6.x();
                    pt.y = obs_world6.y();
                    pt.z = obs_world6.z();
                    cloud_map_.push_back(pt);
                }
            }
        }
    }

    pcl::toROSMsg(cloud_map_, virtual_map_);

    // MODIFIED: 发布动态障碍物位置
    object_msgs::MultiObstacleBoxMsg msg;

    // 移动障碍物1
    object_msgs::ObstacleBoxMsg msg1;
    msg1.x = cylinder1_pos_.x(), msg1.y = cylinder1_pos_.y(), msg1.z = 2.5;  // 中心点
    msg1.x_width = 1.0, msg1.y_width = 1.0, msg1.z_width = 5.0;              // 宽度

    // 移动障碍物2
    object_msgs::ObstacleBoxMsg msg2;
    msg2.x = cylinder2_pos_.x(), msg2.y = cylinder2_pos_.y(), msg2.z = 2.5;  // 中心点
    msg2.x_width = 1.0, msg2.y_width = 1.0, msg2.z_width = 5.0;              // 宽度

    // 移动障碍物3
    object_msgs::ObstacleBoxMsg msg3;
    msg3.x = cylinder3_pos_.x(), msg3.y = cylinder3_pos_.y(), msg3.z = 2.5;  // 中心点
    msg3.x_width = 1.0, msg3.y_width = 1.0, msg3.z_width = 5.0;              // 宽度

    // 移动障碍物4
    object_msgs::ObstacleBoxMsg msg4;
    msg4.x = cylinder4_pos_.x(), msg4.y = cylinder4_pos_.y(), msg4.z = 2.5;  // 中心点
    msg4.x_width = 1.0, msg4.y_width = 1.0, msg4.z_width = 5.0;              // 宽度

    // 移动障碍物5
    object_msgs::ObstacleBoxMsg msg5;
    msg5.x = cylinder5_pos_.x(), msg5.y = cylinder5_pos_.y(), msg5.z = 2.5;  // 中心点
    msg5.x_width = 1.0, msg5.y_width = 1.0, msg5.z_width = 5.0;              // 宽度

    // 移动障碍物6
    object_msgs::ObstacleBoxMsg msg6;
    msg6.x = cylinder6_pos_.x(), msg6.y = cylinder6_pos_.y(), msg6.z = 2.5;  // 中心点
    msg6.x_width = 1.0, msg6.y_width = 1.0, msg6.z_width = 5.0;              // 宽度

    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.box.push_back(msg1);
    msg.box.push_back(msg2);
    msg.box.push_back(msg3);
    msg.box.push_back(msg4);
    msg.box.push_back(msg5);
    msg.box.push_back(msg6);
    multi_obstacle_boxes_pub_.publish(msg);
}

/**
 * @brief 生成包含6个动态障碍物的地图
 */
void VirtualMap::generateSixConstSpeedCylinders() {
    // Clear cloud map vector
    cloud_map_.clear();

    for (double lx = -2.0 - resolution_; lx <= -2.0 + resolution_; lx += resolution_) {
        for (double ly = -5; ly <= 5; ly += resolution_) {
            for (double lz = 0.0; lz <= 5; lz += resolution_) {
                pcl::PointXYZ pt;
                pt.x = lx, pt.y = ly, pt.z = lz;
                cloud_map_.push_back(pt);
            }
        }
    }
    for (double lx = 30.0 - resolution_; lx <= 30.0 + resolution_; lx += resolution_) {
        for (double ly = -5; ly <= 5; ly += resolution_) {
            for (double lz = 0.0; lz <= 5; lz += resolution_) {
                pcl::PointXYZ pt;
                pt.x = lx, pt.y = ly, pt.z = lz;
                cloud_map_.push_back(pt);
            }
        }
    }
    for (double lx = -2; lx <= 30; lx += resolution_) {
        for (double ly = -5 - resolution_; ly <= -5 + resolution_; ly += resolution_) {
            for (double lz = 0.0; lz <= 5; lz += resolution_) {
                pcl::PointXYZ pt;
                pt.x = lx, pt.y = ly, pt.z = lz;
                cloud_map_.push_back(pt);
            }
        }
    }
    for (double lx = -2; lx <= 30; lx += resolution_) {
        for (double ly = 5 - resolution_; ly <= 5 + resolution_; ly += resolution_) {
            for (double lz = 0.0; lz <= 5; lz += resolution_) {
                pcl::PointXYZ pt;
                pt.x = lx, pt.y = ly, pt.z = lz;
                cloud_map_.push_back(pt);
            }
        }
    }

    /* 动态障碍物 */
    const double  dt = 1.0 / update_rate_;
    double        vel {1.0};
    static double vel1 {vel}, vel2 {-vel}, vel3 {vel}, vel4 {-vel}, vel5 {vel}, vel6 {-vel};

    cylinder1_pos_.y() += vel1 * dt;
    cylinder2_pos_.y() += vel2 * dt;
    cylinder3_pos_.y() += vel3 * dt;
    cylinder4_pos_.y() += vel4 * dt;
    cylinder5_pos_.y() += vel5 * dt;
    cylinder6_pos_.y() += vel6 * dt;

    calculateConstObstaclePosition(vel1, cylinder1_pos_.y(), -3.0, 3.0);
    calculateConstObstaclePosition(vel2, cylinder2_pos_.y(), -3.0, 3.0);
    calculateConstObstaclePosition(vel3, cylinder3_pos_.y(), -3.0, 3.0);
    calculateConstObstaclePosition(vel4, cylinder4_pos_.y(), -3.0, 3.0);
    calculateConstObstaclePosition(vel5, cylinder5_pos_.y(), -3.0, 3.0);
    calculateConstObstaclePosition(vel6, cylinder6_pos_.y(), -3.0, 3.0);

    // Face to the cylinder, the cylinder is 1m in radius, 5m in z direction
    Eigen::Vector3d cuboid_size(1.0, 1.0, 5.0);

    for (double lx = -cuboid_size.x() / 2; lx < cuboid_size.x() / 2 + resolution_; lx += resolution_) {
        for (double ly = -cuboid_size.y() / 2; ly < cuboid_size.y() / 2 + resolution_; ly += resolution_) {
            for (double lz = 0.0; lz < cuboid_size.z() + resolution_; lz += resolution_) {
                // In the cylinder
                if (std::pow(lx, 2) + std::pow(ly, 2) < std::pow(0.5, 2)) {
                    Eigen::Vector3d obs_body(lx, ly, lz);
                    Eigen::Vector3d obs_world1, obs_world2, obs_world3, obs_world4, obs_world5, obs_world6;
                    obs_world1 = obs_body + cylinder1_pos_;
                    obs_world2 = obs_body + cylinder2_pos_;
                    obs_world3 = obs_body + cylinder3_pos_;
                    obs_world4 = obs_body + cylinder4_pos_;
                    obs_world5 = obs_body + cylinder5_pos_;
                    obs_world6 = obs_body + cylinder6_pos_;

                    pcl::PointXYZ pt;
                    pt.x = obs_world1.x();
                    pt.y = obs_world1.y();
                    pt.z = obs_world1.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world2.x();
                    pt.y = obs_world2.y();
                    pt.z = obs_world2.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world3.x();
                    pt.y = obs_world3.y();
                    pt.z = obs_world3.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world4.x();
                    pt.y = obs_world4.y();
                    pt.z = obs_world4.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world5.x();
                    pt.y = obs_world5.y();
                    pt.z = obs_world5.z();
                    cloud_map_.push_back(pt);
                    pt.x = obs_world6.x();
                    pt.y = obs_world6.y();
                    pt.z = obs_world6.z();
                    cloud_map_.push_back(pt);
                }
            }
        }
    }

    pcl::toROSMsg(cloud_map_, virtual_map_);

    // MODIFIED: 发布动态障碍物位置
    object_msgs::MultiObstacleBoxMsg msg;

    // 移动障碍物1
    object_msgs::ObstacleBoxMsg msg1;
    msg1.x = cylinder1_pos_.x(), msg1.y = cylinder1_pos_.y(), msg1.z = 2.5;  // 中心点
    msg1.x_width = 1.0, msg1.y_width = 1.0, msg1.z_width = 5.0;              // 宽度

    // 移动障碍物2
    object_msgs::ObstacleBoxMsg msg2;
    msg2.x = cylinder2_pos_.x(), msg2.y = cylinder2_pos_.y(), msg2.z = 2.5;  // 中心点
    msg2.x_width = 1.0, msg2.y_width = 1.0, msg2.z_width = 5.0;              // 宽度

    // 移动障碍物3
    object_msgs::ObstacleBoxMsg msg3;
    msg3.x = cylinder3_pos_.x(), msg3.y = cylinder3_pos_.y(), msg3.z = 2.5;  // 中心点
    msg3.x_width = 1.0, msg3.y_width = 1.0, msg3.z_width = 5.0;              // 宽度

    // 移动障碍物4
    object_msgs::ObstacleBoxMsg msg4;
    msg4.x = cylinder4_pos_.x(), msg4.y = cylinder4_pos_.y(), msg4.z = 2.5;  // 中心点
    msg4.x_width = 1.0, msg4.y_width = 1.0, msg4.z_width = 5.0;              // 宽度

    // 移动障碍物5
    object_msgs::ObstacleBoxMsg msg5;
    msg5.x = cylinder5_pos_.x(), msg5.y = cylinder5_pos_.y(), msg5.z = 2.5;  // 中心点
    msg5.x_width = 1.0, msg5.y_width = 1.0, msg5.z_width = 5.0;              // 宽度

    // 移动障碍物6
    object_msgs::ObstacleBoxMsg msg6;
    msg6.x = cylinder6_pos_.x(), msg6.y = cylinder6_pos_.y(), msg6.z = 2.5;  // 中心点
    msg6.x_width = 1.0, msg6.y_width = 1.0, msg6.z_width = 5.0;              // 宽度

    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.box.push_back(msg1);
    msg.box.push_back(msg2);
    msg.box.push_back(msg3);
    msg.box.push_back(msg4);
    msg.box.push_back(msg5);
    msg.box.push_back(msg6);
    multi_obstacle_boxes_pub_.publish(msg);
}

/**
 * @brief Update the obstacle position
 *
 * @param statu Statu of obstacle
 * @param vel Velocity
 * @param max_vel Maximum velocity
 * @param acc Acceleration
 * @param dt Time interval
 */

void VirtualMap::calculateObstaclePosition(ObstacleStatus& statu,
                                           double&         vel,
                                           const double    max_vel,
                                           const double    acc,
                                           const double    dt) {
    if (statu == ObstacleStatus::ForwardAcc) {
        vel += acc * dt;
        if (vel >= max_vel) {
            vel   = max_vel;
            statu = ObstacleStatus::ForwardDec;
        }
    } else if (statu == ObstacleStatus::ForwardDec) {
        vel -= acc * dt;
        if (vel <= 0.0) {
            vel   = 0.0;
            statu = ObstacleStatus::ReverseAcc;
        }
    } else if (statu == ObstacleStatus::ReverseAcc) {
        vel -= acc * dt;
        if (vel <= -max_vel) {
            vel   = -max_vel;
            statu = ObstacleStatus::ReverseDec;
        }
    } else {
        vel += acc * dt;
        if (vel >= 0.0) {
            vel   = 0.0;
            statu = ObstacleStatus::ForwardAcc;
        }
    }
}

void VirtualMap::publishVirtualMapTimer(const ros::TimerEvent& event) {
    // 根据需要生成点云数据
    if (map_type_ == std::string("complex")) {
        generateComplexObstacles();
    } else if (map_type_ == std::string("six")) {
        generateSixMovingCylinders();
    } else if (map_type_ == std::string("six_const")) {
        generateSixConstSpeedCylinders();
    } else {
        generateFixedWall();
    }

    virtual_map_.header.frame_id = frame_id_;
    virtual_map_pub_.publish(virtual_map_);
}

void VirtualMap::calculateConstObstaclePosition(double& vel, double current_pos, double min_pos, double max_pos) {
    if (current_pos < min_pos || current_pos > max_pos) {
        vel *= -1.0;
    }
}
