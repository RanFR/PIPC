#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <vector>

#include "astar/astar.h"

std::vector<Eigen::Vector3d> pc_vec;
bool                         have_rcv_pc = false;

AstarPtr astar;

ros::Subscriber pc_sub;
ros::Publisher  astar_pub, astar_obs_map_pub;
ros::Timer      timer;

Eigen::Vector3d start_pt(0, 0, 1.0), end_pt(12.0, 0.0, 1.0);

void pubAstarObsMap();

void rcvPointCloudCallback(sensor_msgs::PointCloud2ConstPtr ptr) {
    if (have_rcv_pc) {
        // return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*ptr, *pc_ptr);
    have_rcv_pc = true;
    // std::cerr << pc_ptr->size() << std::endl;

    // 使用CropBox方法进行点云剔除
    pcl::CropBox<pcl::PointXYZ> cb;  // CropBox filter (delete unuseful points)
    // 以记录的里程计的点作为中心点，根据地图的大小设置剔除的点云范围
    cb.setMin(Eigen::Vector4f(-15.0, -15.0, 0.2, 1.0));
    cb.setMax(Eigen::Vector4f(15.0, 15.0, 5.0, 1.0));
    // 设置输入点云为之前记录的静态点云
    cb.setInputCloud(pc_ptr);
    // 进行点云剔除
    cb.filter(*pc_ptr);
    // 将输入的静态点云转化为栅格点云地图
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(0.2, 0.2, 0.2);
    vf.setInputCloud(pc_ptr);
    vf.filter(*pc_ptr);

    // std::cerr << pc_ptr->size() << std::endl;?

    // pc_vec.clear();
    // for (pcl::PointXYZ pc_pt : *pc_ptr) {
    //     pc_vec.push_back(Eigen::Vector3d(pc_pt.x, pc_pt.y, pc_pt.z));
    // }

    // astar->resetGridMap();
    astar->setObsPcl(*pc_ptr, 0.2);
    pubAstarObsMap();
}

void pubAstarObsMap() {
    pcl::PointCloud<pcl::PointXYZ> pcl_pts;

    Eigen::Vector3i gl_size = (Eigen::Vector3d(20, 20, 3) / 0.1).cast<int>();

    for (float x = -15; x <= 15; x += 0.1) {
        for (float y = -15; y <= 15; y += 0.1) {
            for (float z = -0.1; z <= 5.0; z += 0.1) {
                Eigen::Vector3i index = (Eigen::Vector3d(x + 15.0, y + 15.0, z) / 0.1).cast<int>();
                if (astar->isOccupied(index)) {
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

    astar_obs_map_pub.publish(msg);
}

void pubAstarPath(const std::vector<Eigen::Vector3d>& path) {
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp    = ros::Time::now();

    node_vis.ns      = "astar";
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
    astar_pub.publish(node_vis);
}

void execTimer(ros::TimerEvent event) {
    if (!have_rcv_pc) {
        ROS_WARN("Do not receive point cloud.");
        return;
    }

    bool search_flag = astar->searchPath(start_pt, end_pt);

    if (search_flag) {
        std::cerr << "success.\n";
    } else {
        std::cerr << "fail.\n";
    }

    if (search_flag) {
        std::vector<Eigen::Vector3d> astar_path;
        astar->getPath(astar_path);
        pubAstarPath(astar_path);
    }

    astar->reset();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_astar");
    ros::NodeHandle nh("~");

    pc_sub = nh.subscribe("/virtual_map/pointcloud2", 10, &rcvPointCloudCallback);

    astar_pub         = nh.advertise<visualization_msgs::Marker>("test_astar_path", 1);
    astar_obs_map_pub = nh.advertise<sensor_msgs::PointCloud2>("test_astar_grid_map", 10);

    timer = nh.createTimer(ros::Duration(0.01), &execTimer);

    astar = std::make_unique<Astar>();
    Eigen::Vector3d gl_low(-15.0, -15.0, 0.0), gl_upp(15.0, 15.0, 3.0);
    astar->initMap(0.1, gl_low, gl_upp);
    astar->setCenter(Eigen::Vector3d::Zero());

    ros::spin();

    return 0;
}
