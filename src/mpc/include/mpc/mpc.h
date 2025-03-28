#pragma once

#include <ros/ros.h>

#include <Eigen/Core>
#include <algorithm>
#include <memory>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"

class MPC;
using MPCPtr = std::unique_ptr<MPC>;

/* QP formulation:
    min 1/2* x^T H x + f^T x   subject to
    b <= Ax <= b (Ax = b),  d <= Ax <= f,  l <= x <= u
*/
class MpcParameter {
public:
    MpcParameter() = default;

    // 初始状态空间方程矩阵
    Eigen::MatrixXd Ax;
    Eigen::MatrixXd Bx;

    // 由初始状态空间方程转化得到的MPC矩阵
    Eigen::MatrixXd M;
    Eigen::MatrixXd C;

    // 约束权重矩阵
    Eigen::MatrixXd Q_bar;
    Eigen::MatrixXd R_bar, R_con_bar;

    Eigen::VectorXd u_low, u_upp, a_low, a_upp, v_low, v_upp;
    Eigen::VectorXd B_a, B_v, B_p;
    Eigen::MatrixXd A_a, A_v, A_p;
    Eigen::MatrixXd M_a, M_v, M_p;

    Eigen::MatrixXd A_sys;
    Eigen::VectorXd A_sys_low, A_sys_upp;
    Eigen::MatrixXd A_sfc;
    Eigen::VectorXd A_sfc_low, A_sfc_upp;

    Eigen::MatrixXd H;
    Eigen::VectorXd f;
    Eigen::MatrixXd A;
    Eigen::VectorXd Alow, Aupp;

    // OSQP求解所用的稀疏矩阵
    Eigen::SparseMatrix<double> H_sparse;
    Eigen::SparseMatrix<double> A_sparse;

    // 最优控制输入
    Eigen::VectorXd u_optimal;
};

class MPC {
public:
    MPC() = default;

    /* 初始化 */
    void init(ros::NodeHandle& nh);

    /* MPC运行求解 */
    bool run(void);

    /* 模型建立 */
    void generateSystemModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B,
                             double t);  // 系统模型
    void generateMPCModel(const Eigen::MatrixXd& A,
                          const Eigen::MatrixXd& B,
                          Eigen::MatrixXd&       M,
                          Eigen::MatrixXd&       C);  // 将系统模型转为MPC模型
    void problemFormation(void);                // 模型建立

    /* 设置 */
    void setAllConstraint(MpcParameter& mpc_param);                                        // 建立所有约束
    void setGoal(Eigen::Vector3d pr, Eigen::Vector3d vr, Eigen::Vector3d ar, int seg_id);  // 设置目标约束
    void setLinearTerm(MpcParameter&          mpc_param,
                       const Eigen::VectorXd& x_0,
                       const Eigen::VectorXd& x_r);  // 设置线性约束
    void setQuadraticTerm(MpcParameter&          mpc_param,
                          const Eigen::MatrixXd& Q,
                          const Eigen::MatrixXd& R,
                          const Eigen::MatrixXd& R_con,
                          const Eigen::MatrixXd& F);                             // 设置二次约束
    void setSFC(const std::vector<Eigen::Vector3d>& path, double distance);      // 设置飞行走廊约束
    void setStatus(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0);  // 设置当前状态（MPC起始状态）

    // 限制速度和加速度取值在设置范围内
    void limitStatus(Eigen::Vector3d& v0, Eigen::Vector3d& a0);

    // 更新边界条件
    void updateBound(MpcParameter& mpc_param, const Eigen::VectorXd& x_0);

    // 获取最优控制
    void getOptimCmd(Eigen::Vector3d& u, int seg_id);

    // MPC 参数
    int    MPC_HORIZON {5};
    double MPC_STEP {0.1};

    // 初始状态，参考状态
    Eigen::VectorXd X_0_, X_r_;

private:
    // MPC相关参数
    MpcParameter mpc_param_;

    // 打印信息的时间
    ros::Time print_time_;
    int       fps_ {0};

    // MPC 约束相关
    double R_p_ {100.0}, R_v_ {0.0}, R_a_ {0.0}, R_u_ {10.0}, R_u_con_ {1.0}, R_pN_ {0.0}, R_vN_ {0.0}, R_aN_ {0.0};
    // 速度，加速度和控制输入
    Eigen::Vector3d v_min_, v_max_, a_min_, a_max_, u_min_, u_max_;
};
