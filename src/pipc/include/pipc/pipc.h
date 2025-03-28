#pragma once

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
#include <mutex>

#include "astar/astar.h"
#include "dwa/dwa.h"
#include "kinodynamic_astar/kinodynamic_astar.h"
#include "mpc/mpc.h"
#include "obstacles_prediction/obstacles_prediction.h"
#include "polytope/emvp.hpp"
#include "quadrotor_msgs/PositionCommand.h"

enum UAVMode : char { Manual = 0, Hover = 1, Takeoff = 2, Land = 3, Command = 4 };

constexpr double LEAF_SIZE {0.2};  // 栅格地图叶子大小

class PIPC {
public:
    PIPC() = default;
    PIPC(ros::NodeHandle& nh);

    // 发布话题数据
    void pubDWAPath(const std::vector<Eigen::Vector3d>& path);
    void pubCmd(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r);
    void pubMPCPath(const std::vector<Eigen::Vector3d>& pts);
    void pubAttitudeCtrl(const Eigen::Quaterniond& q, const double thrust, const ros::Time& stamp);
    void pubBodyrateCtrl(const Eigen::Vector3d& rate, const double thrust, const ros::Time& stamp);

    // 估算模型
    void computeThrust(const Eigen::Vector3d& acc);
    void convertCommand(Eigen::Vector3d acc, Eigen::Vector3d jerk);
    bool estimateThrustModel(const Eigen::Vector3d& est_a);

    // 回调函数
    void rcvOdomCallback(const nav_msgs::OdometryConstPtr& msg);
    void rcvGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void rcvIMUCallback(const sensor_msgs::ImuConstPtr& msg);
    void rcvLocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void rcvStaticPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void rcvRCInCallback(const mavros_msgs::RCInConstPtr& msg);

    // 定时器函数
    void execTimerCallback(const ros::TimerEvent& event);
    void execReplanTimer(const ros::TimerEvent& event);

    // 重规划路径
    void replanPath();

    // 检查路径是否有效
    bool checkPathFree();

private:
    // 无人机所处模式
    UAVMode mode_;

    // 安全距离
    double safe_dis_ {0.1};

    /* Ros */
    ros::Timer      timer_, replan_timer_;
    ros::Subscriber odom_sub_, goal_sub_, imu_sub_, local_pc_sub_, rc_sub_, static_pc_sub_;
    ros::Publisher  dwa_path_pub_;
    ros::Publisher  gird_map_pub_, cmd_pub_, mpc_path_pub_, px4ctrl_pub_, goal_pub_;
    ros::Time       odom_time_, last_mpc_time_;
    std::mutex odom_mutex_, goal_mutex_, cloud_mutex_, timer_mutex_, replan_timer_mutex_, imu_mutex_, local_pc_mutex_,
        static_pc_mutex_, rc_mutex_;

    /* 点云相关参数 */
    bool has_rcv_pc_msg_ {false};

    bool   simu_flag_, pc_ctrl_flag_, hover_esti_flag_, yaw_ctrl_flag_;
    bool   has_map_flag_ {false}, has_odom_flag_ {false}, replan_flag_ {false}, new_goal_flag_ {false};
    bool   reach_target_ {false};
    int    replan_times_ {0};
    double resolution_;
    double ctrl_delay_;
    double thrust_limit_, hover_perc_;
    double sfc_dis_, path_dis_, expand_dyn_, expand_fix_;
    double ref_dis_;
    int    mpc_ctrl_index_ {0};
    std::vector<Eigen::Vector3d> remain_nodes_;

    std::ofstream       write_time_;
    std::vector<double> log_times_;

    Eigen::Vector3d    goal_pos_, map_low_, map_upp_;
    Eigen::Vector3d    odom_pos_, odom_vel_, odom_acc_, imu_acc_;
    Eigen::Quaterniond odom_quat_, u_quat_;
    Eigen::Vector3d    rate_;
    double             yaw_ {0}, yaw_r_ {0}, yaw_dot_r_ {0}, yaw_gain_;

    int                                                                        dwa_index_ {0};
    std::vector<Eigen::Vector3d>                                               dwa_path_, follow_path_;
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> dwa_tuple_path_;
    std::vector<std::tuple<Eigen::Vector3d, double>>                           dwa_time_path_;
    std::vector<Eigen::Vector3d>                                               replan_path_;
    std::vector<Eigen::Vector3d>                                               local_pc_;
    std::vector<Eigen::Vector3d>                                               static_pc_;

    std::vector<Eigen::Vector3d> mpc_goals_;

    double                                   thr2acc_;
    double                                   thrust_;
    double                                   P_ {100.0};
    Eigen::Vector3d                          Gravity_;
    std::queue<std::pair<ros::Time, double>> timed_thrust_;

    std::deque<pcl::PointCloud<pcl::PointXYZ>> vec_cloud_;
    pcl::PointCloud<pcl::PointXYZ>             static_cloud_;

    /* 所调用的相关类 */
    ObstaclesPredictionPtr obs_pred_ptr_;
    DWAPtr                 dwa_ptr_;
    MPCPtr                 mpc_;
};
