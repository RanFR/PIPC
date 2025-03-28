#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <vector>

#include "kinodynamic_astar/kinodynamic_astar.h"
#include "obstacles_prediction/obstacles_prediction.h"

std::vector<Eigen::Vector3d> pc_vec;
bool                         has_rcv_pc = false;

ObstaclesPredictionPtr obs_pred_ptr;
KinodynamicAstarPtr    kino_astar_ptr;

ros::Subscriber pc_sub;
ros::Publisher  kino_astar_pub, kino_astar_obs_map_pub;
ros::Timer      timer;

Eigen::Vector3d start_pt(0.0, 0.0, 1.0), end_pt(10.0, 0.0, 1.0);

void rcvPointCloudCallback(sensor_msgs::PointCloud2ConstPtr ptr) {
    if (has_rcv_pc) {
        // return;
        // 如果动态地图，注释return即可
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*ptr, *pc_ptr);
    has_rcv_pc = true;

    // 使用CropBox方法进行点云剔除
    pcl::CropBox<pcl::PointXYZ> cb;  // CropBox filter (delete unuseful points)
    // 以记录的里程计的点作为中心点，根据地图的大小设置剔除的点云范围
    cb.setMin(Eigen::Vector4f(-15.0, -15.0, 0.2, 1.0));
    cb.setMax(Eigen::Vector4f(15.0, 15.0, 3.0, 1.0));
    // 设置输入点云为之前记录的静态点云
    cb.setInputCloud(pc_ptr);
    // 进行点云剔除
    cb.filter(*pc_ptr);
    // 将输入的静态点云转化为栅格点云地图
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(pc_ptr);
    vf.filter(*pc_ptr);

    // 将障碍物点云转化为vector形式存储
    pc_vec.clear();
    for (pcl::PointXYZ pc_pt : *pc_ptr) {
        pc_vec.push_back(Eigen::Vector3d(pc_pt.x, pc_pt.y, pc_pt.z));
    }
}

void pubKinoAstarObsMap() {
    pcl::PointCloud<pcl::PointXYZ> pcl_pts;

    for (float x = -15.0; x <= 15.0; x += 0.1) {
        for (float y = -15.0; y <= 15.0; y += 0.1) {
            for (float z = 0.0; z <= 3.0; z += 0.1) {
                if (kino_astar_ptr->testOccupied(Eigen::Vector3d(x, y, z))) {
                    pcl::PointXYZ pcl_pt;
                    pcl_pt.x = x;
                    pcl_pt.y = y;
                    pcl_pt.z = z;
                    pcl_pts.push_back(pcl_pt);
                }
            }
        }
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl_pts, msg);
    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.height          = 1;
    msg.width           = pcl_pts.size();

    kino_astar_obs_map_pub.publish(msg);
}

void pubKinoAstarPath(const std::vector<Eigen::Vector3d>& path) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp    = ros::Time::now();

    node_vis.ns      = "kino_astar";
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
    for (const Eigen::Vector3d& coord : path) {
        pt.x = coord.x();
        pt.y = coord.y();
        pt.z = coord.z();
        node_vis.points.push_back(pt);
    }
    kino_astar_pub.publish(node_vis);
}

void execTimer(ros::TimerEvent e) {
    if (!has_rcv_pc) {
        ROS_WARN("Do not receive point cloud.");
        return;
    }

    // 重置地图障碍后，设置地图障碍
    kino_astar_ptr->resetGridMap();
    kino_astar_ptr->setObsVector(pc_vec, 0.2);
    pubKinoAstarObsMap();

    // Kino搜索
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Eigen::Vector3d oneoo(0.01, 0.0, 0.0);
    kino_astar_ptr->reset();
    KinodynamicAstar::KinoState search_flag =
        kino_astar_ptr->search(start_pt, zero, oneoo, end_pt, zero, ros::Time::now().toSec());
    if (search_flag != KinodynamicAstar::KinoState::NO_PATH) {
        std::vector<Eigen::Vector3d> kino_astar_path = kino_astar_ptr->getPath(0.05);
        std::cout << "Kino path size: " << kino_astar_path.size() << std::endl;
        pubKinoAstarPath(kino_astar_path);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_kino_astar");
    ros::NodeHandle nh("~");

    // 实例化障碍预测
    obs_pred_ptr = std::make_shared<ObstaclesPrediction>();
    obs_pred_ptr->init(nh);

    // 全局点云地图接收
    pc_sub = nh.subscribe("/virtual_map/pointcloud2", 10, &rcvPointCloudCallback);

    // 发布kino astar轨迹和地图
    kino_astar_pub         = nh.advertise<visualization_msgs::Marker>("test_kino_astar_path", 1);
    kino_astar_obs_map_pub = nh.advertise<sensor_msgs::PointCloud2>("test_kino_astar_grid_map", 10);

    // 定时器，运行kinodynamic astar搜索
    timer = nh.createTimer(ros::Duration(0.05), &execTimer);

    // 实例化kinodynamic astar
    kino_astar_ptr = std::make_unique<KinodynamicAstar>();
    // 地图设置
    Eigen::Vector3d gl_low(-15.0, -15.0, 0.0), gl_upp(15.0, 15.0, 3.0);
    kino_astar_ptr->initMap(0.1, gl_low, gl_upp);
    kino_astar_ptr->setCenter(Eigen::Vector3d::Zero());
    kino_astar_ptr->setObstaclesPrediction(obs_pred_ptr);

    ros::spin();

    return 0;
}
