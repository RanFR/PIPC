#ifndef KINODYNAMIC_ASTAR_HPP
#define KINODYNAMIC_ASTAR_HPP

#include <Eigen/Core>
#include <memory>
#include <queue>
#include <tuple>
#include <vector>
// #include <unordered_map>

#include "node/node.h"
#include "obstacles_prediction/obstacles_prediction.h"

constexpr int MAX_ITER_NUM = 10000;

class KinodynamicAstar;
using KinodynamicAstarPtr = std::unique_ptr<KinodynamicAstar>;

class KinodynamicAstar {
  public:
    KinodynamicAstar();
    ~KinodynamicAstar() {};

    // Kinodynamic astar state
    enum KinoState : char {
        REACH_END,
        NEAR_END,
        NO_PATH,
    };

    // 初始化地图
    void initMap(const double resolution, const Eigen::Vector3d& gl_low, const Eigen::Vector3d& gl_upp);
    void setObstaclesPrediction(ObstaclesPredictionPtr ptr);
    // 重置
    void reset(void);
    // 设置中心点
    void setCenter(const Eigen::Vector3d& center);

    // 判断给定点是否在地图中
    bool isInMap(const Eigen::Vector3d& pt);

    // 地图障碍函数
    void resetGridMap();
    void setObs(double coord_x, double coord_y, double coord_z);
    void setObsVector(std::vector<Eigen::Vector3d>& cloud, double radius = 0.2);

    // 将给定点坐标转化为索引
    Eigen::Vector3i coordToGridIndex(const Eigen::Vector3d& pos);

    // 检查地图占用情况
    bool isOccupied(const Eigen::Vector3i& idx);

    // 检查可行性
    bool isPointFeasible(const Eigen::Vector3d& pt, const double time);
    bool isPointFeasible(const std::pair<double, Eigen::Vector3d>& time_pos);
    bool isNodeSampledPathFeasible(const Eigen::Vector3d& cur_pos,
                                   const Eigen::Vector3d& cur_vel,
                                   const double           cur_time,
                                   const Eigen::Vector3d& input,
                                   const double           duration);
    // bool isLineFeasible(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);
    bool isPathFeasible(const std::vector<std::pair<double, Eigen::Vector3d>>& time_path);

    // 搜索Kinodynamic Astar轨迹
    KinoState search(const Eigen::Vector3d& start_pos,
                     const Eigen::Vector3d& start_vel,
                     const Eigen::Vector3d& start_acc,
                     const Eigen::Vector3d& end_pos,
                     const Eigen::Vector3d& end_vel,
                     double                 start_time);

    Eigen::Vector3i posToIndex(const Eigen::Vector3d& pos);
    int             timeToIndex(double time);

    std::vector<double> cubic(double a, double b, double c, double d);
    std::vector<double> quartic(double a, double b, double c, double d, double e);
    double              estimateHeuristic(Eigen::Vector3d start_pos,
                                          Eigen::Vector3d start_vel,
                                          Eigen::Vector3d end_pos,
                                          Eigen::Vector3d end_vel,
                                          double&         optimal_time);
    // Eigen::Matrix<double, 6, 1> stateTransit(const Eigen::Matrix<double, 6, 1> state, Eigen::Vector3d um, double t);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> stateTransit(const Eigen::Vector3d& pos,
                                                              const Eigen::Vector3d& vel,
                                                              const Eigen::Vector3d& input,
                                                              const double           t);

    bool computeShotTraj(const Eigen::Vector3d& start_pos,
                         const Eigen::Vector3d& start_vel,
                         const Eigen::Vector3d& end_pos,
                         const Eigen::Vector3d& end_vel,
                         double                 time_to_goal);

    std::vector<Eigen::Vector3d>                     getPath(void);
    std::vector<Eigen::Vector3d>                     getPath(double time_interval);
    std::vector<std::tuple<double, Eigen::Vector3d>> getTimePath(double time_interval = 0.1);

    /* 用于测试的函数 */
    bool testOccupied(const Eigen::Vector3d& pos);

  private:
    const int       max_node_num_ {1000000};   // 限制单次搜索最大使用节点数
    int             use_node_num_, iter_num_;  // 搜索次数记录
    bool            is_shot_succ_, has_path_;
    double          resolution_, time_resolution_;
    int             check_num_;
    Eigen::Vector3d gl_low_, gl_upp_;
    Eigen::Vector3i gl_size_;
    Eigen::Vector3d center_;

    double start_time_;  // 开始寻路的起始时间
    double lambda_heu_, w_time_;
    double max_tau_;
    double max_vel_, max_acc_;
    double tie_breaker_;

    Eigen::MatrixXd coef_shot_;
    double          t_shot_;

    std::vector<GridState> grid_map_;

    Eigen::Vector3d  end_pos_;
    KinoAstarNodePtr terminate_ptr_;

    NodeHashTable node_hash_table_;  // 用于存放节点信息的哈希表
    std::priority_queue<KinoAstarNodePtr, std::vector<KinoAstarNodePtr>, NodeComparator>
        open_set_;  // Kino Astar搜索使用的开集

    ObstaclesPredictionPtr obs_pred_ptr_;

    //   private:
    //     /* ---------- 主要数据集合 ---------- */
    //     std::vector<KinoAstarNode> path_node_pool_;

    //     NodeHashTable              expanded_nodes_, open_set_hash_, close_set_hash_;

    //     std::vector<KinoAstarNode> path_nodes_;

    //     /* ---------- record data ---------- */
    //     Eigen::Vector3d             start_vel_, end_vel_, start_acc_;

    //     /* ---------- parameter ---------- */
    //     /* search */
    //     double max_tau_, init_max_tau_;
    //     double max_vel_, max_acc_;
    //     double w_time_, horizon_, lambda_heu_;
    //     int    allocate_num_, check_num_;
    //     double tie_breaker_;
    //     bool   optimistic_;

    //     /* map */
    //     Eigen::Vector3d origin_, map_size_3d_;

    //     /* helper */
    //     void retrievePath(KinoAstarNode end_node);

    //     /* shot trajectory */

    //     /* state propagation */
    //     void                        stateTransit(const Eigen::Matrix<double, 6, 1>& state0,
    //                                              Eigen::Matrix<double, 6, 1>&       state1,
    //                                              Eigen::Vector3d                    um,
    //                                              double                             tau);

    //   public:

    //     /* main API */
    //     void      init();
    //     void      setParam(ros::NodeHandle& nh);

    //     std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
    //     void                         getMincoTrajParams(Eigen::VectorXd& times, Eigen::MatrixXd& pts, double delta_t
    //     = 0.1);

    //     void getSamples(double& ts, vector<Eigen::Vector3d>& point_set, vector<Eigen::Vector3d>&
    //     start_end_derivatives);

    //     std::vector<KinoAstarNode> getVisitedNodes();

    //     std::vector<Eigen::Vector3d> getPath();
    //     void                         pubTestKinoPath();
};

#endif
