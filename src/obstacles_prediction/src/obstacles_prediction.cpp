#include "obstacles_prediction/obstacles_prediction.h"

#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

/**
 * @brief 初始化障碍物预测类
 */
ObstaclesPrediction::ObstaclesPrediction() {
    // 初始化
    traj_order_ = 5;                // 贝塞尔轨迹价数
    cps_num_    = traj_order_ + 1;  // 控制点个数

    control_points_.resize(3 * cps_num_);  // XYZ三轴控制点个数

    // 5价贝塞尔曲线的系数矩阵
    M_ = Eigen::MatrixXd::Zero(6, 6);
    M_ << 1, 0, 0, 0, 0, 0, -5, 5, 0, 0, 0, 0, 10, -20, 10, 0, 0, 0, -10, 30, -30, 10, 0, 0, 5, -20, 30, -20, 5, 0, -1,
        5, -10, 10, -5, 1;

    C_ = Eigen::VectorXd::Zero(cps_num_);
    for (int i = 0; i < cps_num_; ++i) {
        C_(i) = factorial(traj_order_) / factorial(i) / factorial(traj_order_ - i);
    }
}

/**
 * @brief 计算阶乘
 */
double ObstaclesPrediction::factorial(int n) {
    // 小于0,返回错误警告
    if (n < 0) {
        ROS_ERROR("Error! Factorial is not defined for negative numbers.");
    };
    // 为0或者1,返回1
    if (n == 0 || n == 1) {
        return 1.0;
    };

    // 计算阶乘
    double res = 1.0;
    for (int i = 2; i <= n; ++i) {
        res *= static_cast<double>(i);
    }
    return res;
}

double ObstaclesPrediction::combinatorial(int n, int m) {
    return factorial(n) / factorial(m) / factorial(n - m);
}

/**
 * @brief 初始化障碍物预测
 */
void ObstaclesPrediction::init(ros::NodeHandle& nh) {
    // 获取预测障碍物的限制速度和加速度数值
    nh.param("obstacles_prediction/limit_vel", limit_vel_, 3.0);
    nh.param("obstacles_prediction/limit_acc", limit_acc_, 6.0);

    // 获取障碍物尺寸话题名称
    nh.param<std::string>("obstacles_prediction/obstacle_box", obstacles_topic_, "obstacles/multi_points");
    // 订阅障碍物尺寸话题
    multi_obstacles_state_sub_ =
        nh.subscribe(obstacles_topic_, 1, &ObstaclesPrediction::multiObstaclesStateCallback, this);

    // TEST
    test_pub_ = nh.advertise<visualization_msgs::Marker>("test_path", 1);
}

/**
 * @brief 初始化障碍物状态。根据输入的障碍物点，设置预测队列的状态数值。
 */
void ObstaclesPrediction::initObstacleStates(const object_msgs::MultiObstacleBoxMsgConstPtr& msg) {
    // 初始化队列
    multi_obstacles_deques_.clear();
    // 清空控制点
    cps_.clear();

    // 获取现在的时间
    double now_time = ros::Time::now().toSec();

    // 初始化多障碍物队列状态以及控制点个数
    for (const object_msgs::ObstacleBoxMsg& pt : msg->box) {
        // 遍历每个障碍物的坐标
        Eigen::Vector4d state(pt.x, pt.y, pt.z, now_time);
        // 初始化障碍物队列为每个障碍物的初始坐标
        std::deque<Eigen::Vector4d> state_deque(DEQUE_LENGTH, state);
        // 传递进入多障碍物队列
        multi_obstacles_deques_.push_back(state_deque);

        // 控制点更新
        Eigen::VectorXd tmp(3 * cps_num_);
        cps_.push_back(tmp);
    }
}

/**
 * @brief 重置贝塞尔曲线参数
 */
void ObstaclesPrediction::reset() {
    pts_.clear();
    time_.clear();
}

/**
 * @brief Predict multi obstacles trajectories
 *
 * @note Consider every obstacle has the same velocity and acceleration
 *
 * @param max_vel Max velocity of the obstacles
 * @param max_acc Max acceleration of the obstacles
 * @param obs_deques Multi obstacles state deque
 *
 * @return true if success, false otherwise
 */
bool ObstaclesPrediction::multiPredict(const double                             max_vel,
                                       const double                             max_acc,
                                       std::vector<std::deque<Eigen::Vector4d>> obs_deques) {
    /* Consider every obstacle has the same stamp */
    /* Bezier curve parameters */
    start_time_ = obs_deques[0][0][3];
    last_time_  = obs_deques[0][DEQUE_LENGTH - 1][3];
    total_time_ = last_time_ + TIME_INTERVAL * PREDICT_POINTS_NUM;
    time_scale_ = (last_time_ - start_time_) + TIME_INTERVAL * PREDICT_POINTS_NUM;

    max_vel_ = max_vel;
    max_acc_ = max_acc;

    for (size_t i = 0; i < obs_num_; ++i) {
        // 重置障碍物信息
        reset();

        std::deque<Eigen::Vector4d>& obs_deque = obs_deques[i];
        for (int j = 0; j < DEQUE_LENGTH; ++j) {
            // obs_deque: x, y, z, time. Four parameters
            pts_.push_back(obs_deque[j].head(3));
            time_.push_back(obs_deque[j][3]);
        }

        /* Solve the QP problem and get the result */
        if (!solveMultiOsqp(i)) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 计算对应时间和位置下的不确定度。不确定度越大，说明对应时间和位置不可确定，应该尽可能避免这种情况
 *
 * @param time_pt 给定点对应的时间，全局时间
 * @param cur_pos 当前点
 *
 * @return 不确定度的数值大小，越小证明越可信
 * @retval -1(HIT_UNCERTAINTY)，表示碰撞
 */
double ObstaclesPrediction::calculateUncertainty(double cur_time, const Eigen::Vector3d& cur_pos) {
    // 当前时间小于记录障碍物的起始时间，认为不确定度为0.0
    if (cur_time < start_time_) {
        return 0.0;
    }

    // 正则化预测的时间，转化为类中的局部时间
    double u = (cur_time - start_time_) / time_scale_;

    // 正则化项超过轨迹总时间，不可预测为0.0
    if (u > 1.0) {
        return 0.0;
    }

    // 获取当前时刻的增长率
    const int RATE_NUMBER = static_cast<int>((cur_time - last_time_) / UNCERTAINTY_TIME_STEP)
                            + 1;  // 每时间步长增长一次不确定度，至少增加一次不确定度
    const double RATE = std::pow(GROWTH_RATE, RATE_NUMBER);  // 最终的增长比率

    double uncertainty_value {0.0};
    // 遍历每一条障碍物的预测轨迹
    for (size_t i = 0; i < cps_.size(); ++i) {
        Eigen::Vector3d traj_pt = getBezierPt(i, cur_time);

        // TODO: 没有考虑z轴
        Eigen::Vector2d delta_xy       = (cur_pos.head(2) - traj_pt.head(2));
        double          delta_distance = delta_xy.norm();
        Eigen::Vector2d outer_width {RATE * box_ptr_->box[i].x_width / 2.0, RATE * box_ptr_->box[i].y_width / 2.0};
        double          outer_distance = outer_width.norm();
        // 距离中心距离小于外接圆半径加安全距离，认为碰撞
        if (delta_distance < outer_distance + OP_SAFE_DISTANCE) {
            return HIT_UNCERTAINTY;
        } else if (delta_distance < outer_distance + OP_MAX_SAFE_DISTANCE) {
            uncertainty_value += std::pow(delta_distance - outer_distance, 2);
        } else {
            uncertainty_value += 0.0;
        }
    }

    return uncertainty_value;
}

/**
 * @brief 获得给定时间对应的第idx条内塞尔曲线上的点
 *
 * @param t_now 输入当前时间
 *
 * @return 返回Eigen::Vector3d形式的点
 */
Eigen::Vector3d ObstaclesPrediction::getBezierPt(int idx, double t_now) {
    // 获得相对于贝塞尔时间尺度的时间
    double u = (t_now - start_time_) / time_scale_;

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    // x, y, z三轴
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < cps_num_; ++j) {
            pos(i) += cps_[idx](cps_num_ * i + j) * C_(j) * std::pow(u, j) * std::pow(1.0 - u, traj_order_ - j);
        }
    }
    return pos;
}

/**
 * @brief 获得给定时间间隔下，采样的贝塞尔曲线点
 */
std::vector<Eigen::Vector3d> ObstaclesPrediction::getSamplePtsFromBezier(double sample_interal) {
    std::vector<Eigen::Vector3d> sample_pts;
    for (double t = start_time_; t < total_time_; t += sample_interal) {
        // TODO:后续增加对不同轨迹的选择逻辑
        Eigen::Vector3d pt = getBezierPt(0, t);
        sample_pts.push_back(pt);
    }

    return sample_pts;
}

/**
 * @brief 获得给定时间间隔下，采样的预测部分的贝塞尔曲线点
 */
std::vector<Eigen::Vector3d> ObstaclesPrediction::getSamplePredictedPtsFromBezier(int idx, double sample_interval) {
    std::vector<Eigen::Vector3d> sample_pts;
    for (double t = last_time_ - 1e-3; t < total_time_; t += sample_interval) {
        Eigen::Vector3d pt = getBezierPt(idx, t);
        sample_pts.push_back(pt);
    }

    return sample_pts;
}

/**
 * @brief 获得预测的多条贝塞尔曲线上的预测点
 */
std::vector<std::vector<Eigen::Vector3d>> ObstaclesPrediction::getMulitSamplePredictedPtsFromBezier(
    double sample_interval) {
    std::vector<std::vector<Eigen::Vector3d>> sample_path;
    sample_path.resize(obs_num_);

    // 遍历所有障碍点
    for (int i = 0; i < obs_num_; ++i) {
        sample_path.push_back(getSamplePredictedPtsFromBezier(i, sample_interval));
    }

    return sample_path;
}

/**
 * @brief 计算贝塞尔曲线的Bn
 */
Eigen::VectorXd ObstaclesPrediction::getBnOfBezier(const double& t_now) {
    // 调整时间尺度
    double          u  = (t_now - start_time_) / time_scale_;
    Eigen::VectorXd bn = Eigen::VectorXd::Zero(cps_num_);
    // 计算对应控制点的bn
    for (int i = 0; i < cps_num_; ++i) {
        bn(i) = combinatorial(traj_order_, i) * std::pow(u, i) * std::pow(1 - u, traj_order_ - i);
    }
    return bn;
}

/**
 * @brief Judge the given point and corresponding time is occupied or not
 *
 * @param time_pt The given point and corresponding time
 *
 * @return true if occupied, false otherwise
 */
bool ObstaclesPrediction::isMultiOccupied(double cur_time, const Eigen::Vector3d& cur_pos) {
    // Current time is less than start time of trajectory, consider it as not occupied.
    // 当前时间小于轨迹的起始时间，认为没有被占用
    if (cur_time < start_time_) {
        return false;
    }

    // Normalized time in part of prediction
    // 正则化预测的时间
    double u = (cur_time - start_time_) / time_scale_;  // time span

    // Current time is more than final time of trajectory, consider it as not occupied.
    // 正则化项超过轨迹总时间，认为没有占用
    if (u > 1.0) {
        // The time is out of range, consider it as not occupied
        return false;
    }

    // Loop every obstacle trajectory
    // 遍历每一条障碍物的预测轨迹
    for (const Eigen::VectorXd& coeff : cps_) {
        // Calculate the point based on Bezier curve
        // 根据贝塞尔曲线公式和时间计算对应的轨迹点
        Eigen::Vector3d traj_pt = Eigen::Vector3d::Zero();
        for (int i = 0; i < 3; ++i) {  // x, y, z
            for (int j = 0; j < cps_num_; ++j) {
                traj_pt(i) += coeff(cps_num_ * i + j) * C_(j) * std::pow(u, j) * std::pow(1.0 - u, traj_order_ - j);
            }
        }

        // 仅检查X轴和Y轴
        // TODO:目前设定安全距离为0.2,半径为0.5米，后续更新为不确定度的方式
        // MODIFIED:新建汉书calculateUncertainty中实现，目前本函数用于实现kinodynamic搜索
        if (std::pow(traj_pt.x() - cur_pos.x(), 2) + std::pow(traj_pt.y() - cur_pos.y(), 2) < 0.49) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 获取约束中的正则项
 */
Eigen::MatrixXd ObstaclesPrediction::getRegularizationMatrix() {
    double          tmp_scale    = std::pow(time_scale_, 3);
    Eigen::MatrixXd coeff_matrix = Eigen::MatrixXd::Zero(cps_num_, cps_num_);
    for (int i = 2; i < cps_num_; ++i) {
        for (int j = 2; j < cps_num_; ++j) {
            coeff_matrix(i, j) = i * (i - 1) * j * (j - 1) * std::pow(1.0, i + j - 1) / (i + j - 1) / tmp_scale;
        }
    }
    Eigen::MatrixXd regularization_matrix = M_.transpose() * coeff_matrix * M_;
    return regularization_matrix;
}

/**
 * @brief 获取约束中的线性项
 */
Eigen::MatrixXd ObstaclesPrediction::getResidualMatrix() {
    Eigen::MatrixXd residual_matrix = Eigen::MatrixXd::Zero(DEQUE_LENGTH, DEQUE_LENGTH);
    for (int i = 0; i < DEQUE_LENGTH - 1; ++i) {
        residual_matrix(i, i) = std::tanh(1.0 / (last_time_ - time_[i]));
    }
    residual_matrix(DEQUE_LENGTH - 1, DEQUE_LENGTH - 1) = 1.0;

    return residual_matrix;
}

/**
 * @brief 二次规划中的P矩阵
 */
Eigen::MatrixXd ObstaclesPrediction::getP() {
    int             var_number                                   = cps_num_ + DEQUE_LENGTH;
    Eigen::MatrixXd P_part                                       = Eigen::MatrixXd::Zero(var_number, var_number);
    P_part.block(0, 0, cps_num_, cps_num_)                       = getRegularizationMatrix();
    P_part.block(cps_num_, cps_num_, DEQUE_LENGTH, DEQUE_LENGTH) = getResidualMatrix();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3 * var_number, 3 * var_number);
    for (int i = 0; i < 3; ++i) {
        P.block(i * var_number, i * var_number, var_number, var_number) = P_part;
    }

    return P;
}

/**
 * @brief 二次规划中的q向量
 */
Eigen::VectorXd ObstaclesPrediction::getq() {
    int             var_number = cps_num_ + DEQUE_LENGTH;
    Eigen::VectorXd q          = Eigen::VectorXd::Zero(3 * var_number);
    return q;
}

/**
 * @brief 二次规划中的A矩阵
 */
Eigen::MatrixXd ObstaclesPrediction::getA() {
    /* Constraints */
    // position constraints
    Eigen::MatrixXd Ayc = Eigen::MatrixXd::Zero(DEQUE_LENGTH, cps_num_);
    Eigen::MatrixXd Ay  = -Eigen::MatrixXd::Identity(DEQUE_LENGTH, DEQUE_LENGTH);
    for (int i = 0; i < DEQUE_LENGTH; ++i) {
        Ayc.row(i) = getBnOfBezier(time_[i]).transpose();
    }
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(DEQUE_LENGTH, cps_num_ + DEQUE_LENGTH);
    tmp << Ayc, Ay;
    double          var_number = cps_num_ + DEQUE_LENGTH;
    Eigen::MatrixXd Apt        = Eigen::MatrixXd::Zero(3 * DEQUE_LENGTH, 3 * var_number);
    for (int i = 0; i < 3; ++i) {
        Apt.block(i * DEQUE_LENGTH, i * var_number, DEQUE_LENGTH, var_number) = tmp;
    }

    // velocity constraints, only consider the axis of x, paper mentioned
    Eigen::MatrixXd Avel = Eigen::MatrixXd::Zero(cps_num_ - 1, cps_num_);
    for (int i = 0; i < cps_num_ - 1; ++i) {
        Avel(i, i)     = -traj_order_ / time_scale_;
        Avel(i, i + 1) = traj_order_ / time_scale_;
    }

    // acceleration constraints, also only consider the axis of x
    Eigen::MatrixXd Aacc = Eigen::MatrixXd::Zero(cps_num_ - 2, cps_num_);
    for (int i = 0; i < cps_num_ - 2; ++i) {
        double tmp_scale = pow(time_scale_, 2);
        Aacc(i, i)       = traj_order_ * (traj_order_ - 1) / tmp_scale;
        Aacc(i, i + 1)   = -2 * traj_order_ * (traj_order_ - 1) / tmp_scale;
        Aacc(i, i + 2)   = traj_order_ * (traj_order_ - 1) / tmp_scale;
    }

    // Merge the constraints
    Eigen::MatrixXd A                     = Eigen::MatrixXd::Zero(Apt.rows() + Avel.rows() + Aacc.rows(), Apt.cols());
    A.block(0, 0, Apt.rows(), Apt.cols()) = Apt;
    A.block(Apt.rows(), 0, Avel.rows(), Avel.cols())               = Avel;
    A.block(Apt.rows() + Avel.rows(), 0, Aacc.rows(), Aacc.cols()) = Aacc;

    return A;
}

/**
 * @brief 二次规划中的l向量
 */
Eigen::VectorXd ObstaclesPrediction::getl() {
    // position lower bound
    Eigen::VectorXd lpt = Eigen::VectorXd::Zero(3 * DEQUE_LENGTH);
    for (int i = 0; i < DEQUE_LENGTH; ++i) {
        lpt(i)                    = pts_[i](0);
        lpt(i + DEQUE_LENGTH)     = pts_[i](1);
        lpt(i + 2 * DEQUE_LENGTH) = pts_[i](2);
    }

    // velocity lower bound
    Eigen::VectorXd lvel = Eigen::VectorXd::Ones(cps_num_ - 1) * -max_vel_;

    // acceleration lower bound
    Eigen::VectorXd lacc = Eigen::VectorXd::Ones(cps_num_ - 2) * -max_acc_;

    // Merge the lower bound
    Eigen::VectorXd l = Eigen::VectorXd::Zero(lpt.rows() + lvel.rows() + lacc.rows());
    l << lpt, lvel, lacc;

    return l;
}

/**
 * @brief 二次规划中的u向量
 */
Eigen::VectorXd ObstaclesPrediction::getu() {
    // position upper bound
    Eigen::VectorXd upt = Eigen::VectorXd::Zero(3 * DEQUE_LENGTH);
    for (int i = 0; i < DEQUE_LENGTH; ++i) {
        upt(i)                    = pts_[i](0);
        upt(i + DEQUE_LENGTH)     = pts_[i](1);
        upt(i + 2 * DEQUE_LENGTH) = pts_[i](2);
    }

    // velocity upper bound
    Eigen::VectorXd uvel = Eigen::VectorXd::Ones(cps_num_ - 1) * max_vel_;

    // acceleration upper bound
    Eigen::VectorXd uacc = Eigen::VectorXd::Ones(cps_num_ - 2) * max_acc_;

    // Merge the upper bound
    Eigen::VectorXd u = Eigen::VectorXd::Zero(upt.rows() + uvel.rows() + uacc.rows());
    u << upt, uvel, uacc;

    return u;
}

/**
 * @brief 求解二次规划问题
 */
bool ObstaclesPrediction::solveOsqp() {
    // Get the matrix for OSQP
    Eigen::MatrixXd P = getP();
    Eigen::VectorXd q = getq();
    Eigen::MatrixXd A = getA();
    Eigen::VectorXd l = getl();
    Eigen::VectorXd u = getu();

    // Convert the dense matrix to sparse matrix
    Eigen::SparseMatrix<double> P_sparse = P.sparseView();
    Eigen::SparseMatrix<double> A_sparse = A.sparseView();

    // Initialize the OSQP solver
    OsqpEigen::Solver solver;

    // Settings
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    // Set the initial data of the OSQP
    solver.data()->setNumberOfConstraints(A.rows());
    solver.data()->setNumberOfVariables(P.rows());
    if (!solver.data()->setHessianMatrix(P_sparse)) {
        ROS_ERROR("Failed to set hessian matrix");
        return false;
    }
    if (!solver.data()->setGradient(q)) {
        ROS_ERROR("Failed to set gradient");
        return false;
    }
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
        ROS_ERROR("Failed to set linear constraints matrix");
        return false;
    }
    if (!solver.data()->setLowerBound(l)) {
        ROS_ERROR("Failed to set lower bound");
        return false;
    }
    if (!solver.data()->setUpperBound(u)) {
        ROS_ERROR("Failed to set upper bound");
        return false;
    }

    // Instantiate the solver
    if (!solver.initSolver()) {
        ROS_ERROR("Failed to initialize the solver");
        return false;
    }

    // Solve the problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        ROS_ERROR("Failed to solve the problem");
        return false;
    }

    // Get the result
    Eigen::VectorXd result     = solver.getSolution();
    double          var_number = cps_num_ + DEQUE_LENGTH;
    for (int i = 0; i < 3; ++i) {
        control_points_.segment(i * cps_num_, cps_num_) = result.segment(i * var_number, cps_num_);
    }

    return true;
}

/**
 * @brief Get the multi predicted trajectories of obstacles
 *
 * @param idx The index of each obstacle
 *
 * @return true if success, false otherwise
 */
bool ObstaclesPrediction::solveMultiOsqp(size_t idx) {
    // Get the matrix for OSQP
    Eigen::MatrixXd P = getP();
    Eigen::VectorXd q = getq();
    Eigen::MatrixXd A = getA();
    Eigen::VectorXd l = getl();
    Eigen::VectorXd u = getu();

    // Convert the dense matrix to sparse matrix
    Eigen::SparseMatrix<double> P_sparse = P.sparseView();
    Eigen::SparseMatrix<double> A_sparse = A.sparseView();

    // Initialize the OSQP solver
    OsqpEigen::Solver solver;

    // Settings
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    // Set the initial data of the OSQP
    solver.data()->setNumberOfConstraints(A.rows());
    solver.data()->setNumberOfVariables(P.rows());
    if (!solver.data()->setHessianMatrix(P_sparse)) {
        ROS_ERROR("Failed to set hessian matrix");
        return false;
    }
    if (!solver.data()->setGradient(q)) {
        ROS_ERROR("Failed to set gradient");
        return false;
    }
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
        ROS_ERROR("Failed to set linear constraints matrix");
        return false;
    }
    if (!solver.data()->setLowerBound(l)) {
        ROS_ERROR("Failed to set lower bound");
        return false;
    }
    if (!solver.data()->setUpperBound(u)) {
        ROS_ERROR("Failed to set upper bound");
        return false;
    }

    // Instantiate the solver
    if (!solver.initSolver()) {
        ROS_ERROR("Failed to initialize the solver");
        return false;
    }

    // Solve the problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        ROS_ERROR("Failed to solve the problem");
        return false;
    }

    // Get the result
    Eigen::VectorXd result     = solver.getSolution();
    int             var_number = cps_num_ + DEQUE_LENGTH;
    for (int i = 0; i < 3; ++i) {
        cps_[idx].segment(i * cps_num_, cps_num_) = result.segment(i * var_number, cps_num_);
    }

    return true;
}

/**
 * @brief Callback function to get the multi-obstacles state
 *
 * @param msg. The multi-obstacles state.
 */
void ObstaclesPrediction::multiObstaclesStateCallback(const object_msgs::MultiObstacleBoxMsgConstPtr& msg) {
    // 线程锁
    obstacle_prediction_mutex_.lock();

    // 更新障碍物状态信息
    box_ptr_ = msg;

    // 如果读取的障碍物点数与上一次不同，则重新开始预测
    obs_num_ = msg->box.size();
    if (obs_num_ != last_obs_num_) {
        // 初始化障碍物状态
        initObstacleStates(msg);

        // 更新障碍物个数信息
        last_obs_num_ = obs_num_;

        // 解锁
        obstacle_prediction_mutex_.unlock();

        // 初始更新不进行预测
        return;
    }

    // 更新每个障碍最新的状态
    for (int i = 0; i < obs_num_; ++i) {
        std::deque<Eigen::Vector4d>& obs_deque = multi_obstacles_deques_[i];

        Eigen::Vector4d state;
        state << msg->box[i].x, msg->box[i].y, msg->box[i].z, msg->header.stamp.toSec();
        obs_deque.push_back(state);

        // Maintain the deque, keep the number of elements in the deque to be DEQUE_LENGTH
        if (obs_deque.size() > DEQUE_LENGTH) {
            obs_deque.pop_front();
        }
    }

    // Multi predictions
    // 多障碍物预测
    if (!multiPredict(limit_vel_, limit_acc_, multi_obstacles_deques_)) {
        ROS_WARN("[Target Prediction] Multi prediction failed!");
        obstacle_prediction_mutex_.unlock();
        return;
    }

    // 测试第一条预测的轨迹
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp    = ros::Time::now();

    node_vis.ns      = "obstacle_prediction";
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

    geometry_msgs::Point         pt;
    std::vector<Eigen::Vector3d> test_pts = getSamplePredictedPtsFromBezier(0, 0.1);
    for (const Eigen::Vector3d& coord : test_pts) {
        pt.x = coord.x();
        pt.y = coord.y();
        pt.z = coord.z();
        node_vis.points.push_back(pt);
    }
    test_pub_.publish(node_vis);

    obstacle_prediction_mutex_.unlock();
}
