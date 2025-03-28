#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <tuple>
#include <vector>

#include "astar/astar.h"
#include "node/node.h"
#include "obstacles_prediction/obstacles_prediction.h"
#include "util/tools.hpp"

class DWA;
using DWAPtr = std::unique_ptr<DWA>;

constexpr int    MAX_TREE_DEPTH = 32;    // 最大的搜索树深度
constexpr double DECLINE_RATE   = 0.99;  // 代价衰减率

constexpr int MAX_LAYER_NODE_NUM = 1000;  // 每一层最大存储的节点数

constexpr double MIN_SAFE_DISTANCE = 0.25;  // 设定的最小安全距离
constexpr double MAX_COST_DISTANCE = 3.0;   // 设定的最大代价距离
constexpr double HIT_COST          = -1.0;  // 设定的碰撞代价，使用负数表示

constexpr double POSE_INDEX_RESOLUTION = 0.5;  // 位置索引分辨率
constexpr double TIME_INDEX_RESOLUTION = 0.1;  // 时间索引分辨率

// Jerk空间采样
constexpr int SAMPLE_JERK_NUMBER {9};

class DWA {
public:
    DWA() = default;

    // 初始化类
    void init(ros::NodeHandle& nh);

    void reset();

    /* 内部地图相关函数 */
    // 设置地图中心点
    void setMapCenter(const Eigen::Vector3d& center);
    void setMapParameters(double resolution, double map_x_size, double map_y_size, double map_z_size);
    // 设置地图信息
    void setMap(const pcl::PointCloud<pcl::PointXYZ>& pcl_map);
    void setStaticMap(const pcl::PointCloud<pcl::PointXYZ>& static_pcl_map);

    /* 搜索 */
    std::vector<std::vector<DWANodePtr>> search(const Eigen::Vector3d& start_pos,
                                                const Eigen::Vector3d& start_vel,
                                                const Eigen::Vector3d& start_acc,
                                                const Eigen::Vector3d& end_pos,
                                                const Eigen::Vector3d& end_vel,
                                                double                 start_time,
                                                int                    given_depth = MAX_TREE_DEPTH);

    /* 获取最优路径 */
    std::vector<DWANodePtr>      getBestNodePath(const std::vector<std::vector<DWANodePtr>>& tree);  // 节点路径
    std::vector<Eigen::Vector3d> getBestPath(const std::vector<std::vector<DWANodePtr>>& tree);      // 位置路径
    std::vector<std::tuple<Eigen::Vector3d, double>> getBestTimePath(
        const std::vector<std::vector<DWANodePtr>>& tree);  // 包含当前时间的路径
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> getBestTuplePath(
        const std::vector<std::vector<DWANodePtr>>& tree);  // 组合路径

    /* 用于估计启发式函数数值以及配套函数 */
    double estimateHeuristic(const Eigen::Vector3d& start_pos,
                             const Eigen::Vector3d& start_vel,
                             const Eigen::Vector3d& end_pos,
                             const Eigen::Vector3d& end_vel);
    // 求解三次方程根
    std::vector<double> cubic(double a, double b, double c, double d);
    // 求解四次方程根
    std::vector<double> quartic(double a, double b, double c, double d, double e);

    // 状态空间扩展
    std::vector<DWANodePtr> stateExpand(const DWANodePtr& node, int sample_num);  // 默认三轴采样数量一致

    /* 位置索引和时间索引 */
    int posToIndex(const Eigen::Vector3d& pos);
    int timeToIndex(double time);

    // 计算节点代价
    double calCost(const Eigen::Vector3d& cur_pos,
                   const Eigen::Vector3d& cur_vel,
                   const Eigen::Vector3d& end_pos,
                   const Eigen::Vector3d& end_vel);
    double calStaticObstacleCost(const Eigen::Vector3d& pos);

    bool isOccupied(const Eigen::Vector3d& pos);

    // 设置障碍预测类指针
    void setObstaclePrediction(ObstaclesPredictionPtr ptr);

private:
    /* 局部地图相关的参数 */
    std::vector<DWANodePtr> dwa_node_ptr_vec_;  // 节点指针索引
    Eigen::Vector3d         center_;
    Eigen::Vector3d         low_bound_, upp_bound_;
    Eigen::Vector3i         map_size_;
    double                  resolution_;
    /* 全局地图参数 */
    double global_map_size_x_min_, global_map_size_x_max_, global_map_size_y_min_, global_map_size_y_max_,
        global_map_size_z_min_, global_map_size_z_max_;

    /* KD树相关参数 */
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_flann_;                       // 存储整个地图的kd树
    pcl::KdTreeFLANN<pcl::PointXYZ> static_kd_tree_flann_;                // 存储静态地图的KD树
    std::mutex        kd_tree_flann_mutex_, static_kd_tree_flann_mutex_;  // KD树线程锁，确保读取操作中不会进行修改
    std::atomic<bool> current_kd_tree_flann_ {false};                     // 选择KD树的索引，true使用a，false使用b

    double start_time_;
    double max_vel_ {2.0}, max_acc_ {3.0}, max_jerk_ {10.0};
    double lambda_heu_ {1.0}, w_time_ {1.0};

    // 权重
    double weight_reference_point_distance_ {1.0}, weight_heuristic_distance_ {1.0},
        weight_static_obstacle_distance_ {1.0}, weight_dynamic_obstacle_uncertainty_ {1.0},
        weight_future_position_ {1.0};

    // 存储每一轴最大的位置索引数值
    Eigen::Vector3i pos_max_idx_;

    // 时间步长，默认0.1s
    double time_step_ {0.1};
    // 采样
    int sample_num_ {8};

    /* 调用的其他类 */
    ObstaclesPredictionPtr obs_pred_ptr_;
    AstarPtr               astar_ptr_, next_astar_ptr_;
};
