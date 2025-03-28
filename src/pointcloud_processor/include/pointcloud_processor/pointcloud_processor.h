#pragma once

/* 根据动态障碍物的预测位置获得近似的静态地图 */

#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <memory>

#include "object_msgs/MultiObstacleBoxMsg.h"

constexpr float PI = 3.1415927;

class PointCloudProcessor;
using PointCloudProcessorPtr = std::unique_ptr<PointCloudProcessor>;

class PointCloudProcessor {
  public:
    PointCloudProcessor() = default;

    // 初始化
    void init(ros::NodeHandle& nh);

    // 接受局部点云的回调函数
    void rcvLocalMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    // 接收动态障碍物位置的回调函数
    void rcvDynamicObstacleBoxCallback(const object_msgs::MultiObstacleBoxMsgConstPtr& msg);

    // 发布静态地图
    void publishStaticMap(const ros::TimerEvent& event);

  private:
    ros::Timer static_map_timer_;
    ros::Subscriber local_map_sub_, dynamic_obstacles_box_sub_;  // 接受无人机获得的局部地图，动态障碍物位置
    ros::Publisher static_map_pub_;                              // 处理后的静态地图

    // 存储局部点云信息
    pcl::PointCloud<pcl::PointXYZ> local_map_;
    pcl::PointCloud<pcl::PointXYZ> static_map_;
};
