#include "pointcloud_processor/pointcloud_processor.h"

/**
 * @brief 初始化类
 */
void PointCloudProcessor::init(ros::NodeHandle& nh) {
    // 接受无人机观测获得的局部地图
    std::string local_map_topic;
    nh.param<std::string>("pointcloud_processor/local_map", local_map_topic, "local_map");
    std::cerr << local_map_topic << std::endl;
    local_map_sub_ = nh.subscribe(local_map_topic, 10, &PointCloudProcessor::rcvLocalMapCallback, this);

    // 接收动态障碍物位置
    std::string obstacle_box_topic;
    nh.param<std::string>("pointcloud_processor/obstacle_box", obstacle_box_topic, "obstacles_prediction/multi_points");
    dynamic_obstacles_box_sub_ =
        nh.subscribe(obstacle_box_topic, 1, &PointCloudProcessor::rcvDynamicObstacleBoxCallback, this);

    // 发布处理后的点云地图
    std::string static_map_topic;
    nh.param<std::string>("pointcloud_processor/static_map", static_map_topic, "static_map");
    static_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(static_map_topic, 10, true);  // 静态地图，不包含动态障碍物

    // 定时发布处理的静态地图
    static_map_timer_ = nh.createTimer(ros::Duration(0.1), &PointCloudProcessor::publishStaticMap, this);
}

/**
 * @brief 接受无人机观测的局部点云的回调函数，存储点云数据
 */
void PointCloudProcessor::rcvLocalMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 转化为pcl形式存储
    pcl::fromROSMsg(*msg, local_map_);
}

/**
 * @brief 接收动态障碍物的位置和具体的header（包含时间）
 */
void PointCloudProcessor::rcvDynamicObstacleBoxCallback(const object_msgs::MultiObstacleBoxMsgConstPtr& msg) {
    static_map_.clear();
    if (!msg->box.empty()) {
        for (const pcl::PointXYZ& pcl_pt : local_map_) {
            bool in_box = false;
            for (const object_msgs::ObstacleBoxMsg& box : msg->box) {
                // 点在中心为xyz的长方体内部，跳过
                if (std::abs(box.x - pcl_pt.x) <= box.x_width / 2.0 + 0.1
                    && std::abs(box.y - pcl_pt.y) <= box.y_width / 2.0 + 0.1
                    && std::abs(box.z - pcl_pt.z) <= box.z_width / 2.0 + 0.1) {
                    in_box = true;
                    break;
                }
            }

            // 如果在box中，跳过当次循环
            if (in_box) {
                continue;
            }

            // 插入新地图
            static_map_.push_back(pcl_pt);
        }
    }
}

/**
 * @brief 定时发布处理后的静态地图
 */
void PointCloudProcessor::publishStaticMap(const ros::TimerEvent& event) {
    sensor_msgs::PointCloud2 static_map;
    pcl::toROSMsg(static_map_, static_map);

    static_map.header.frame_id = "world";
    static_map.header.stamp    = ros::Time::now();

    static_map_pub_.publish(static_map);
}
