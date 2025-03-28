#ifndef ASTAR_H
#define ASTAR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <queue>
#include <vector>

#include "node/node.h"

constexpr double PI {3.14159265358979323846};

class Astar;
using AstarPtr = std::unique_ptr<Astar>;

class Astar {
public:
    // 初始化A*地图
    void initMap(const double resolution, const Eigen::Vector3d& gl_low, const Eigen::Vector3d& gl_upp);

    // 重置地图节点
    void reset(void);

    // 设置中心点
    void setCenter(const Eigen::Vector3d& center);

    // 判断给定点是否在地图中
    bool isInMap(const Eigen::Vector3d& pt);

    // 为地图设置障碍
    void setObs(double coord_x, double coord_y, double coord_z);
    void setObsPcl(const pcl::PointCloud<pcl::PointXYZ>& cloud, double radius = 0.2);
    void setObsVector(std::vector<Eigen::Vector3d>& cloud, double radius = 0.2);

    // 获取占据地图
    void getOccupiedPcl(pcl::PointCloud<pcl::PointXYZ>& cloud);

    // 检查可行性
    bool isPointFeasible(const Eigen::Vector3d& pt);                            // 给定点
    bool isLineFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);  // 给定两点组成的线段
    bool isPathFeasible(const std::vector<Eigen::Vector3d>& path);              // 给定线段组成的总路径

    // 坐标转换
    Eigen::Vector3d gridIndexToCoord(const Eigen::Vector3i& index);
    Eigen::Vector3i coordToGridIndex(const Eigen::Vector3d& pt);

    // 搜索A星路径
    virtual bool searchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    // 获得A星路径
    void                         getPath(std::vector<Eigen::Vector3d>& path);
    std::vector<Eigen::Vector3d> getPath();
    // 简化A星路径
    void simplifyPath(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint);
    // Floyd方法
    void floydHandle(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint);

    bool isOccupied(int idx_x, int idx_y, int idx_z);
    bool isOccupied(const Eigen::Vector3i index);

private:
    /* 占据相关函数 */

    bool isFree(int idx_x, int idx_y, int idx_z);
    bool isFree(const Eigen::Vector3i index);

    // 计算启发式函数值
    double calcHeu(AstarNodePtr node1, AstarNodePtr node2);
    // 获得邻居节点
    void getNeighbors(AstarNodePtr cur, std::vector<AstarNodePtr>& neighbors, std::vector<double>& costs);

    double          resolution_;
    Eigen::Vector3i gl_size_;
    Eigen::Vector3d gl_low_, gl_upp_;
    Eigen::Vector3d center_;

    // 栅格地图
    // enum GridState : char { FREE, OCCUPIED };
    std::vector<GridState> grid_map_;

    std::vector<std::vector<std::vector<AstarNodePtr>>>                          astar_node_map_;
    Eigen::Vector3d                                                              start_pt_, end_pt_;
    AstarNodePtr                                                                 terminate_ptr_;
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, NodeComparator> open_lists_;
};

#endif
