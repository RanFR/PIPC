#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

// 自定义消息
#include "object_msgs/MultiObstacleBoxMsg.h"

// 预先编译
constexpr int    DEQUE_LENGTH {50};        // 记录过往50个障碍物位置点
constexpr int    PREDICT_POINTS_NUM {32};  // 预测未来32个障碍物位置点
constexpr double TIME_INTERVAL {0.1};      // 记录障碍点和预测障碍点的时间间隔
constexpr double SAMPLE_INTERVALS {0.1};   // 输出轨迹的采样时间间隔

constexpr double HIT_UNCERTAINTY {-1.0};       // 确认碰撞的不确定度
constexpr double UNCERTAINTY_TIME_STEP {0.1};  // 不确定度增长的时间步长
constexpr double GROWTH_RATE {1.025};          // 不确定度增长率
constexpr double MAX_UNCERTAINTY {1e6};        // 最大的不确定度数值

constexpr double OP_MAX_SAFE_DISTANCE {3.0};  //障碍物预测的最大安全距离
constexpr double OP_SAFE_DISTANCE {0.25};     // 障碍物预测的安全距离

// 障碍物预测
class ObstaclesPrediction;
using ObstaclesPredictionPtr = std::shared_ptr<ObstaclesPrediction>;

class ObstaclesPrediction {
public:
    ObstaclesPrediction();

    // 阶乘
    double factorial(int n);
    double combinatorial(int n, int m);

    // 初始化
    void init(ros::NodeHandle& nh);
    void initObstacleStates(const object_msgs::MultiObstacleBoxMsgConstPtr& msg);  // 初始化障碍物状态
    // 重置障碍物位置信息和对应的时间信息
    void reset();

    // 预测函数
    bool multiPredict(const double                             max_vel,
                      const double                             max_acc,
                      std::vector<std::deque<Eigen::Vector4d>> obs_deques);  // 对多个动态障碍物进行预测

    // 计算不确定度
    double calculateUncertainty(double cur_time, const Eigen::Vector3d& cur_pos);

    // 获得给定时间对应的第idx条贝塞尔曲线上的点
    Eigen::Vector3d getBezierPt(int idx, double t_now);
    // 采样给定时间间隔下贝塞尔曲线上的点
    std::vector<Eigen::Vector3d> getSamplePtsFromBezier(double sample_interval = 0.1);  // 第idx条贝塞尔轨迹
    std::vector<Eigen::Vector3d> getSamplePredictedPtsFromBezier(
        int    idx,
        double sample_interval = 0.5);  // 仅预测的第idx条贝塞尔曲线轨迹点
    // 获取预测的多条贝塞尔曲线上的点
    std::vector<std::vector<Eigen::Vector3d>> getMulitSamplePredictedPtsFromBezier(double sample_interval = 0.5);

    // 贝塞尔曲线对应的bc
    Eigen::VectorXd getBnOfBezier(const double& t_now);

    // 检查是否碰撞
    bool isMultiOccupied(double cur_time, const Eigen::Vector3d& cur_pos);

    // OSQP求解器
    Eigen::MatrixXd getRegularizationMatrix();
    Eigen::MatrixXd getResidualMatrix();
    Eigen::MatrixXd getP();
    Eigen::VectorXd getq();
    Eigen::MatrixXd getA();
    Eigen::VectorXd getl();
    Eigen::VectorXd getu();
    bool            solveOsqp();
    bool            solveMultiOsqp(size_t idx);

    /* Ros函数 */
    void multiObstaclesStateCallback(const object_msgs::MultiObstacleBoxMsgConstPtr& msg);  // 多个障碍物位置

private:
    // 线程锁
    std::mutex obstacle_prediction_mutex_;

    Eigen::VectorXd              control_points_;  // 贝塞尔曲线控制点
    std::vector<Eigen::VectorXd> cps_;             // 多段贝塞尔曲线组成的控制点
    Eigen::MatrixXd              M_, Q_;           //
    Eigen::VectorXd              C_;

    int traj_order_, cps_num_;  // 贝塞尔曲线价数和控制点个数

    // Point and corresponding time
    std::vector<Eigen::Vector3d> pts_;
    std::vector<double>          time_;
    double                       start_time_ {ros::Time::now().toSec()}, last_time_;
    double                       total_time_;
    double                       time_scale_;

    // Constraints of velocity and acceleration
    double max_vel_, max_acc_;
    double limit_vel_, limit_acc_;

    pcl::PointCloud<pcl::PointXYZ> cloud_;

    object_msgs::MultiObstacleBoxMsgConstPtr box_ptr_;

    int                                      obs_num_, last_obs_num_ {-1};
    std::vector<std::deque<Eigen::Vector4d>> multi_obstacles_deques_;

    /* Ros related */
    std::string     obstacles_topic_;
    ros::Publisher  obstacles_pcd_pub_;  // default cylinder
    ros::Subscriber obstacles_state_sub_, multi_obstacles_state_sub_;

    // TEST
    ros::Publisher test_pub_;
};
