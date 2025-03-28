
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "dwa/dwa.h"
#include "obstacles_prediction/obstacles_prediction.h"

bool has_rcv_pc {false};

bool start_search {false};

DWAPtr                 dwa_ptr;
ObstaclesPredictionPtr obs_pred_ptr;

ros::Subscriber pc_sub, trigger_sub;
ros::Publisher  dwa_pub, last_layer_pc_pub;
ros::Timer      timer;

Eigen::Vector3d start_pt(0.0, 0.0, 1.0), end_pt(12.0, 0.0, 1.0);
Eigen::Vector3d start_vel, start_acc;

void rcvTriggerCallback(geometry_msgs::PoseStampedConstPtr ptr) {
    start_search = true;
}

void rcvPointCloudCallback(sensor_msgs::PointCloud2ConstPtr ptr) {
    has_rcv_pc = true;

    pcl::PointCloud<pcl::PointXYZ> pcl_msg;
    pcl::fromROSMsg(*ptr, pcl_msg);

    // 使用CropBox方法进行点云剔除
    pcl::CropBox<pcl::PointXYZ> cb;  // CropBox filter (delete unuseful points)
    // 以记录的里程计的点作为中心点，根据地图的大小设置剔除的点云范围
    cb.setMin(Eigen::Vector4f(-20.0, -20.0, 0.2, 1.0));
    cb.setMax(Eigen::Vector4f(20.0, 20.0, 3.0, 1.0));
    // 设置输入点云为之前记录的静态点云
    cb.setInputCloud(pcl_msg.makeShared());
    // 进行点云剔除
    cb.filter(pcl_msg);
    // 将输入的静态点云转化为栅格点云地图
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(pcl_msg.makeShared());
    vf.filter(pcl_msg);

    if (pcl_msg.size() == 0) {
        pcl_msg.push_back({-20.0, -20.0, 0.0});
    }

    // 获取KD树
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann;
    kd_tree_flann.setInputCloud(pcl_msg.makeShared());
    dwa_ptr->setKdTreeFlann(kd_tree_flann);

    dwa_ptr->setMapCenter(start_pt);
}

void pubDWAPath(const std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>& tuple_path) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp    = ros::Time::now();

    node_vis.ns      = "dwa";
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.type               = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action             = visualization_msgs::Marker::ADD;
    node_vis.id                 = 0;
    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.scale.x = 0.1;
    node_vis.scale.y = 0.1;
    node_vis.scale.z = 0.1;

    geometry_msgs::Point pt;
    for (auto tuple_state : tuple_path) {
        auto [pos, vel, acc] = tuple_state;
        pt.x                 = pos.x();
        pt.y                 = pos.y();
        pt.z                 = pos.z();
        node_vis.points.push_back(pt);
    }
    dwa_pub.publish(node_vis);
}

void pubLastLayerPC(const std::vector<std::vector<DWANodePtr>>& tree) {
    // 找到最后一层
    // std::vector<DWANodePtr> last_layer = tree.back();
    // ROS_INFO("Last layer size: %zu", last_layer.size());

    // 存入pcl中
    pcl::PointCloud<pcl::PointXYZ> pcl_pc;
    for (auto last_layer : tree) {
        for (DWANodePtr ptr : last_layer) {
            Eigen::Vector3d pos = ptr->pos;
            pcl::PointXYZ   pt(pos.x(), pos.y(), pos.z());
            pcl_pc.push_back(pt);
        }
    }

    // 转化为ros消息
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl_pc, msg);

    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.width           = pcl_pc.size();
    msg.height          = 1;

    // ROS_INFO("PC size: %zu", pcl_pc.size());

    last_layer_pc_pub.publish(msg);
}

void execTimer(ros::TimerEvent event) {
    if (!has_rcv_pc) {
        ROS_WARN("Do not receive point cloud.");
        return;
    }

    if (!start_search) {
        ROS_INFO("Wait for start trigger!");
        return;
    }

    // DWA搜索
    Eigen::Vector3d                      zero = Eigen::Vector3d::Zero();
    auto                                 t1   = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<DWANodePtr>> tree =
        dwa_ptr->search(start_pt, start_vel, start_acc, end_pt, zero, ros::Time::now().toSec());
    auto                          t2       = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = t2 - t1;
    // std::cerr << "Duration: " << duration.count() << std::endl;
    // std::cerr << "Current vel: " << start_vel.transpose() << std::endl;
    if (tree.size() > 1) {
        // 说明搜索树至少2层深度，说明有效
        std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> tuple_path =
            dwa_ptr->getBestTuplePath(tree);
        pubDWAPath(tuple_path);
        pubLastLayerPC(tree);
        // auto tuple_state = tuple_path[0];
        auto tuple_state     = tuple_path[1];
        auto [pos, vel, acc] = tuple_state;
        start_pt = pos, start_vel = vel, start_acc = acc;
    } else {
        ROS_WARN("DWA failed. The depth of tree is only 1!!!");
        start_vel << 0.0, 0.0, 0.0;
        start_acc << 0.0, 0.0, 0.0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dwa");
    ros::NodeHandle nh("~");

    // 全局点云地图接收
    pc_sub = nh.subscribe("/virtual_map/static_map", 10, &rcvPointCloudCallback);

    // 触发器
    trigger_sub = nh.subscribe("/move_base_simple/goal", 1, &rcvTriggerCallback);

    // 发布kino astar轨迹和地图
    dwa_pub           = nh.advertise<visualization_msgs::Marker>("test_dwa_path", 1);
    last_layer_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("test_last_layer_pc", 10);

    // 定时器，运行kinodynamic astar搜索
    timer = nh.createTimer(ros::Duration(0.1), &execTimer);

    // 实例化动障预测
    obs_pred_ptr = std::make_shared<ObstaclesPrediction>();
    obs_pred_ptr->init(nh);

    // 实例化DWA
    dwa_ptr = std::make_unique<DWA>();
    dwa_ptr->setObstaclePrediction(obs_pred_ptr);

    // 地图设置
    dwa_ptr->init(nh);
    dwa_ptr->setMapCenter(Eigen::Vector3d::Zero());

    ros::spin();

    return 0;
}
