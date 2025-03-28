#include "mpc/mpc.h"

void MPC::init(ros::NodeHandle& nh) {
    // MPC参数
    nh.param("mpc/horizon", MPC_HORIZON, 5);
    nh.param("mpc/step", MPC_STEP, 0.1);

    // MPC约束参数
    nh.param("mpc/R_p", R_p_, 100.0);
    nh.param("mpc/R_v", R_v_, 0.0);
    nh.param("mpc/R_a", R_a_, 0.0);
    nh.param("mpc/R_u", R_u_, 10.0);
    nh.param("mpc/R_u_con", R_u_con_, 1.0);
    nh.param("mpc/R_pN", R_pN_, 0.0);
    nh.param("mpc/R_vN", R_vN_, 0.0);
    nh.param("mpc/R_aN", R_aN_, 0.0);

    // MPC约束中的速度限制
    nh.param("mpc/vx_min", v_min_.x(), -1.0);
    nh.param("mpc/vy_min", v_min_.y(), -1.0);
    nh.param("mpc/vz_min", v_min_.z(), -1.0);
    nh.param("mpc/vx_max", v_max_.x(), 1.0);
    nh.param("mpc/vy_max", v_max_.y(), 1.0);
    nh.param("mpc/vz_max", v_max_.z(), 1.0);

    // MPC约束中的加速度限制
    nh.param("mpc/ax_min", a_min_.x(), -1.0);
    nh.param("mpc/ay_min", a_min_.y(), -1.0);
    nh.param("mpc/az_min", a_min_.z(), -1.0);
    nh.param("mpc/ax_max", a_max_.x(), 1.0);
    nh.param("mpc/ay_max", a_max_.y(), 1.0);
    nh.param("mpc/az_max", a_max_.z(), 1.0);

    // MPC约束中的输入（加加速度）限制
    nh.param("mpc/ux_min", u_min_.x(), -1.0);
    nh.param("mpc/uy_min", u_min_.y(), -1.0);
    nh.param("mpc/uz_min", u_min_.z(), -1.0);
    nh.param("mpc/ux_max", u_max_.x(), 1.0);
    nh.param("mpc/uy_max", u_max_.y(), 1.0);
    nh.param("mpc/uz_max", u_max_.z(), 1.0);

    // 对问题进行建模
    problemFormation();

    // 初始状态和参考状态
    X_0_.resize(mpc_param_.M.cols(), 1);
    X_r_.resize(mpc_param_.M.rows(), 1);
}

/**
 * @brief 运行MPC程序
 *
 * @retval true/false
 * @return 成功运行或者失败的标志
 */
bool MPC::run(void) {
    // MPC运行初始时间
    ros::Time mpc_start_run_time = ros::Time::now();

    // 设置线性项
    setLinearTerm(mpc_param_, X_0_, X_r_);

    // 更新系统状态和约束
    updateBound(mpc_param_, X_0_);

    // TEST:取消SFC约束
    bool cancel_sfc = true;
    if (cancel_sfc) {
        mpc_param_.A_sfc.resize(0, mpc_param_.A_sfc.cols());
        mpc_param_.A_sfc_low.resize(0);
        mpc_param_.A_sfc_upp.resize(0);
    }

    // 更新安全飞行走廊的约束
    mpc_param_.A_sfc_low -= mpc_param_.B_p.head(mpc_param_.A_sfc_low.size());
    mpc_param_.A_sfc_upp -= mpc_param_.B_p.head(mpc_param_.A_sfc_upp.size());

    // 生成所有不等式约束，将SFC约束添加到线性约束中
    Eigen::Index A_sys_rows = mpc_param_.A_sys.rows(), A_sys_cols = mpc_param_.A_sys.cols();
    Eigen::Index A_sfc_rows = mpc_param_.A_sfc.rows(), A_sfc_cols = mpc_param_.A_sfc.cols();
    mpc_param_.A.resize(A_sys_rows + A_sfc_rows, A_sys_cols);
    // A sys
    mpc_param_.A.block(0, 0, A_sys_rows, A_sys_cols) = mpc_param_.A_sys;
    // A sfc
    mpc_param_.A.block(A_sys_rows, 0, A_sfc_rows, A_sfc_cols) = mpc_param_.A_sfc;
    // Alow
    Eigen::Index A_sys_low_size = mpc_param_.A_sys_low.size(), A_sfc_low_size = mpc_param_.A_sfc_low.size();
    mpc_param_.Alow.resize(A_sys_low_size + A_sfc_low_size);
    // Alow sys
    mpc_param_.Alow.segment(0, A_sys_low_size) = mpc_param_.A_sys_low;
    // Alow sfc
    mpc_param_.Alow.segment(A_sys_low_size, A_sfc_low_size) = mpc_param_.A_sfc_low;
    // Aupp
    Eigen::Index A_sys_upp_size = mpc_param_.A_sys_upp.size(), A_sfc_upp_size = mpc_param_.A_sfc_upp.size();
    mpc_param_.Aupp.resize(A_sys_upp_size + A_sfc_upp_size);
    // Aupp sys
    mpc_param_.Aupp.segment(0, A_sys_upp_size) = mpc_param_.A_sys_upp;
    // Aupp sfc
    mpc_param_.Aupp.segment(A_sys_upp_size, A_sfc_upp_size) = mpc_param_.A_sfc_upp;

    // 将约束矩阵和海森矩阵转化为稀疏矩阵
    mpc_param_.H_sparse = mpc_param_.H.sparseView();
    mpc_param_.A_sparse = mpc_param_.A.sparseView();

    // 重置SFC约束
    mpc_param_.A_sfc.resize(0, 0);
    mpc_param_.A_sfc_low.resize(0, 1);
    mpc_param_.A_sfc_upp.resize(0, 1);

    // 使用OsqpEigen求解MPC问题
    OsqpEigen::Solver solver;
    // solver.settings()->setTimeLimit(0.008);
    solver.settings()->setVerbosity(false);  // 不显示详细的求解器内容
    solver.settings()->setWarmStart(true);   // 暖启动
    solver.data()->setNumberOfConstraints(mpc_param_.A_sparse.rows());
    solver.data()->setNumberOfVariables(mpc_param_.f.rows());
    solver.data()->setHessianMatrix(mpc_param_.H_sparse);
    solver.data()->setGradient(mpc_param_.f);
    solver.data()->setLinearConstraintsMatrix(mpc_param_.A_sparse);
    solver.data()->setLowerBound(mpc_param_.Alow);
    solver.data()->setUpperBound(mpc_param_.Aupp);

    // 初始化求解器
    bool init_flag  = solver.initSolver();
    bool solve_flag = true;
    // 初始化成功，则进行求解
    if (init_flag) {
        // if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError
            || solver.getStatus() != OsqpEigen::Status::Solved) {
            solve_flag = false;
        }
    } else {
        ROS_ERROR("[MPC]: Can't set mpc problem!");
        solve_flag = false;
    }

    // 增加FPS记录
    ++fps_;

    // 初始化成功，同时求解成功
    if (solve_flag && init_flag) {
        // 计算最优输入
        mpc_param_.u_optimal = solver.getSolution();

        // 打印当前数据
        double cur_vel = X_0_.segment(3, 3).norm();
        // 每秒打印一次
        if ((ros::Time::now() - print_time_).toSec() > 1.0) {
            print_time_ = ros::Time::now();
            std::cout << "MPC fps: " << fps_
                      << ", this time is: " << (ros::Time::now() - mpc_start_run_time).toSec() * 1000
                      << " ms. Velocity now is: " << cur_vel << "m/s. " << std::endl;
            fps_ = 0;
        }
        // 返回成功标志
        return true;
    } else {
        /* 初始化或者求解不成功 */
        // 如果初始化成功，而求解失败，输出失败原因
        // TODO:后续可以写为一个函数的形式

        OsqpEigen::Status status = solver.getStatus();
        if (status == OsqpEigen::Status::DualInfeasibleInaccurate) {
            ROS_ERROR("[MPC]: Error status: Dual Infeasible Inaccurate");
        }
        if (status == OsqpEigen::Status::PrimalInfeasibleInaccurate) {
            ROS_ERROR("[MPC]: Error status: Primal Infeasible Inaccurate");
        }
        if (status == OsqpEigen::Status::PrimalInfeasible) {
            ROS_ERROR("[MPC]: Error status: Primal Infeasible");
        }
        if (status == OsqpEigen::Status::SolvedInaccurate) {
            ROS_ERROR("[MPC]: Error status: Solved Inaccurate");
        }
        if (status == OsqpEigen::Status::Sigint) {
            ROS_ERROR("[MPC]: Error status: Sigint");
        }
        if (status == OsqpEigen::Status::MaxIterReached) {
            ROS_ERROR("[MPC]: Error status: Max Iter Reached");
        }
        if (status == OsqpEigen::Status::DualInfeasible) {
            ROS_ERROR("[MPC]: Error status: Dual Infeasible");
        }
        if (status == OsqpEigen::Status::NonCvx) {
            ROS_ERROR("[MPC]: Error status: NonCvx");
        }

        return false;
    }
}

/**
 * @brief 对原始系统进行建模：x(n+1)=Ax(n)+Bu(n)
 *
 * @param A 系统状态矩阵A
 * @param B 系统状态矩阵B
 * @param t 离散时间
 */
void MPC::generateSystemModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, double t) {
    /*
        根据间隔时间t，设置模型
        x = [px, py, pz, vx, vy, vz, ax, ay, az]^T
        u = [jx, jy, jz]^T
        x(t+1) = Ax(t) + Bu(t)
    */
    A                   = Eigen::MatrixXd::Zero(9, 9);
    A.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * t;
    A.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity() * t * t * 0.5;
    A.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();
    A.block(3, 6, 3, 3) = Eigen::Matrix3d::Identity() * t;
    A.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity();

    B                   = Eigen::MatrixXd::Zero(9, 3);
    B.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 1.0 / 6.0 * std::pow(t, 3);
    B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * 1.0 / 2.0 * std::pow(t, 2);
    B.block(6, 0, 3, 3) = Eigen::Matrix3d::Identity() * t;
}

/**
 * @brief 根据单段的模型，设定HORIZON长度下的模型，建立MPC系统
 *
 * @param A 状态空间矩阵A
 * @param B 状态空间矩阵B
 * @param M MPC转化矩阵A得到的状态矩阵M
 * @param C MPC转化矩阵A和矩阵B得到的控制矩阵C
 */
void MPC::generateMPCModel(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, Eigen::MatrixXd& M, Eigen::MatrixXd& C) {
    /*
        计算HORIZON过程中的M矩阵和C矩阵
        x(1) = Ax(0) + Bu(0)
        x(2) = Ax(1) + Bu(1) = A^2x(0) + Bu(1) + ABu(0)
        ...
        x(t) = Ax(t-1) + bu(t-1) = A^tx(0) + Bu(t-1) + ABu(t-2) + ... + A^(t-1)Bu(0)
        其中M矩阵即为x对应的矩阵，C矩阵为u对应的矩阵
    */

    // 重置矩阵M和矩阵C
    M = Eigen::MatrixXd::Zero(MPC_HORIZON * A.rows(), A.cols());
    C = Eigen::MatrixXd::Zero(MPC_HORIZON * B.rows(), MPC_HORIZON * B.cols());

    // 根据HORIZON计算对应的M矩阵分块和C矩阵分块
    Eigen::MatrixXd tmp_A = Eigen::MatrixXd::Identity(A.rows(), A.cols());
    for (int i = 0; i < MPC_HORIZON; ++i) {
        if (i == 0) {
            // 初始C矩阵为B
            C.block(0, 0, B.rows(), B.cols()) = B;
        } else {
            // 计算C矩阵中，各个u对应的矩阵数值
            Eigen::MatrixXd tmp_C = Eigen::MatrixXd(B.rows(), C.cols());
            tmp_C << tmp_A * B, C.block((i - 1) * B.rows(), 0, B.rows(), B.cols() * (MPC_HORIZON - 1));
            C.block(B.rows() * i, 0, B.rows(), C.cols()) = tmp_C;
        }

        tmp_A                                        = tmp_A * A;
        M.block(A.rows() * i, 0, A.rows(), A.cols()) = tmp_A;
    }
}

/**
 * @brief 对问题进行建模
 */
void MPC::problemFormation(void) {
    /*
        system status: {p1, v1, a1, p2, v2, a2, ... , pN, vN, aN}
        input: {u0, u1, u2, ... , u(N-1)}
    */

    // 系统模型
    generateSystemModel(mpc_param_.Ax, mpc_param_.Bx, MPC_STEP);
    generateMPCModel(mpc_param_.Ax, mpc_param_.Bx, mpc_param_.M, mpc_param_.C);

    // 计算代价矩阵（二次项和线性项）
    Eigen::MatrixXd Q     = Eigen::MatrixXd::Zero(9, 9);
    Q.block(0, 0, 3, 3)   = Eigen::Matrix3d::Identity() * R_p_;
    Q.block(3, 3, 3, 3)   = Eigen::Matrix3d::Identity() * R_v_;
    Q.block(6, 6, 3, 3)   = Eigen::Matrix3d::Identity() * R_a_;
    Eigen::MatrixXd R     = Eigen::MatrixXd::Identity(3, 3) * R_u_;
    Eigen::MatrixXd R_con = Eigen::MatrixXd::Identity(3, 3) * R_u_con_;
    Eigen::MatrixXd F     = Eigen::MatrixXd::Zero(9, 9);
    F.block(0, 0, 3, 3)   = Eigen::Matrix3d::Identity() * R_pN_;
    F.block(3, 3, 3, 3)   = Eigen::Matrix3d::Identity() * R_vN_;
    F.block(6, 6, 3, 3)   = Eigen::Matrix3d::Identity() * R_aN_;
    // 设置二次项和线性项的代价矩阵
    setQuadraticTerm(mpc_param_, Q, R, R_con, F);
    setLinearTerm(mpc_param_, Eigen::VectorXd(9, 1).setZero(), Eigen::VectorXd(9 * MPC_HORIZON, 1).setZero());
    // 设置所有约束
    setAllConstraint(mpc_param_);

    // 重置最优控制
    mpc_param_.u_optimal.resize(mpc_param_.f.rows());

    ROS_INFO("\033[1;32mMPC problem formation done!\033[0m");
}

/**
 * @brief 设置所有的约束
 */
void MPC::setAllConstraint(MpcParameter& mpc_param) {
    /* 约束中的系统矩阵和对应的约束范围 */
    mpc_param.A_sys.resize(3 * 3 * MPC_HORIZON, 3 * MPC_HORIZON);  // 3轴，3种状态（位置、速度和加速度）
    mpc_param.A_sys_low.resize(3 * 3 * MPC_HORIZON);               // 行数和A_sys对应，列数为1
    mpc_param.A_sys_upp.resize(3 * 3 * MPC_HORIZON);

    /* 系统约束，包含输入、速度和加速度约束*/
    mpc_param.u_low.resize(3 * MPC_HORIZON);
    mpc_param.u_upp.resize(3 * MPC_HORIZON);
    mpc_param.v_low.resize(3 * MPC_HORIZON);
    mpc_param.v_upp.resize(3 * MPC_HORIZON);
    mpc_param.a_low.resize(3 * MPC_HORIZON);
    mpc_param.a_upp.resize(3 * MPC_HORIZON);

    // 遍历Horizon，设置约束
    for (int i = 0; i < MPC_HORIZON; ++i) {
        mpc_param.u_low.segment(i * u_min_.rows(), u_min_.rows()) = u_min_;
        mpc_param.u_upp.segment(i * u_max_.rows(), u_max_.rows()) = u_max_;
        mpc_param.v_low.segment(i * v_min_.rows(), v_min_.rows()) = v_min_;
        mpc_param.v_upp.segment(i * v_max_.rows(), v_max_.rows()) = v_max_;
        mpc_param.a_low.segment(i * a_min_.rows(), a_min_.rows()) = a_min_;
        mpc_param.a_upp.segment(i * a_max_.rows(), a_max_.rows()) = a_max_;
    }

    /* 计算矩阵A_p, A_v, A_a：将系统状态矩阵p、v、a转化为输入u */
    mpc_param.A_p.resize(3 * MPC_HORIZON, mpc_param.C.cols());
    mpc_param.A_v.resize(3 * MPC_HORIZON, mpc_param.C.cols());
    mpc_param.A_a.resize(3 * MPC_HORIZON, mpc_param.C.cols());
    mpc_param.M_p.resize(3 * MPC_HORIZON, mpc_param.M.cols());
    mpc_param.M_v.resize(3 * MPC_HORIZON, mpc_param.M.cols());
    mpc_param.M_a.resize(3 * MPC_HORIZON, mpc_param.M.cols());
    mpc_param.B_a.resize(mpc_param.M_p.rows(), 1);
    mpc_param.B_v.resize(mpc_param.M_v.rows(), 1);
    mpc_param.B_p.resize(mpc_param.M_a.rows(), 1);

    // 遍历Horizon，计算转化矩阵
    for (int i = 0; i < MPC_HORIZON; ++i) {
        mpc_param.A_p.block(3 * i, 0, 3, mpc_param.A_p.cols()) = mpc_param.C.block(9 * i + 0, 0, 3, mpc_param.C.cols());
        mpc_param.A_v.block(3 * i, 0, 3, mpc_param.A_v.cols()) = mpc_param.C.block(9 * i + 3, 0, 3, mpc_param.C.cols());
        mpc_param.A_a.block(3 * i, 0, 3, mpc_param.A_a.cols()) = mpc_param.C.block(9 * i + 6, 0, 3, mpc_param.C.cols());
        mpc_param.M_p.block(3 * i, 0, 3, mpc_param.M_p.cols()) = mpc_param.M.block(9 * i + 0, 0, 3, mpc_param.M.cols());
        mpc_param.M_v.block(3 * i, 0, 3, mpc_param.M_v.cols()) = mpc_param.M.block(9 * i + 3, 0, 3, mpc_param.M.cols());
        mpc_param.M_a.block(3 * i, 0, 3, mpc_param.M_a.cols()) = mpc_param.M.block(9 * i + 6, 0, 3, mpc_param.M.cols());
    }

    // 约束矩阵
    Eigen::MatrixXd A_u                                 = Eigen::MatrixXd::Identity(3 * MPC_HORIZON, 3 * MPC_HORIZON);
    mpc_param.A_sys.block(0, 0, A_u.rows(), A_u.cols()) = A_u;
    mpc_param.A_sys.block(A_u.rows(), 0, mpc_param.A_v.rows(), mpc_param.A_v.cols()) = mpc_param.A_v;
    mpc_param.A_sys.block(A_u.rows() + mpc_param.A_v.rows(), 0, mpc_param.A_a.rows(), mpc_param.A_a.cols()) =
        mpc_param.A_a;

    // 更新边界
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(9);
    updateBound(mpc_param, x_0);
}

/**
 * @brief 设置参考的终点目标
 *
 * @param pr 参考的位置
 * @param vr 参考的速度
 * @param ar 参考的加速度
 * @param step
 */
void MPC::setGoal(Eigen::Vector3d pr, Eigen::Vector3d vr, Eigen::Vector3d ar, int seg_id) {
    if (seg_id > MPC_HORIZON || seg_id < 0) {
        ROS_WARN("[MPC]: Check goal index! Error index: %d", seg_id);
        return;
    }

    // 将给定状态放入一列中，依次为位置、速度和加速度
    Eigen::VectorXd x_0(X_0_.rows());
    x_0.segment(0, pr.rows())                     = pr;
    x_0.segment(pr.rows(), vr.rows())             = vr;
    x_0.segment(pr.rows() + vr.rows(), ar.rows()) = ar;

    // 设置参考状态
    X_r_.segment(x_0.rows() * seg_id, x_0.rows()) = x_0;
}

/**
 * @brief 设置MPC求解问题中的线性项
 */
void MPC::setLinearTerm(MpcParameter& mpc_param, const Eigen::VectorXd& x_0, const Eigen::VectorXd& x_r) {
    if (x_r.rows() != mpc_param.M.rows()) {
        ROS_ERROR("[MPC]: MPC linear term set goal error!");
        return;
    }

    // 设置线性项
    mpc_param.f.resize(mpc_param.C.rows(), 1);
    mpc_param.f =
        ((x_0.transpose() * mpc_param.M.transpose() - x_r.transpose()) * mpc_param.Q_bar * mpc_param.C).transpose();
}

/**
 * @brief 设置MPC求解问题中的二次项代价
 */
void MPC::setQuadraticTerm(MpcParameter&          mpc_param,
                           const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R,
                           const Eigen::MatrixXd& R_con,
                           const Eigen::MatrixXd& F) {
    mpc_param.Q_bar.resize(Q.rows() * MPC_HORIZON, Q.cols() * MPC_HORIZON);
    mpc_param.Q_bar.setZero();
    mpc_param.R_bar.resize(R.rows() * MPC_HORIZON, R.cols() * MPC_HORIZON);
    mpc_param.R_bar.setZero();
    mpc_param.R_con_bar.resize(R_con.rows() * MPC_HORIZON, R_con.cols() * MPC_HORIZON);
    mpc_param.R_con_bar.setZero();
    for (int i = 0; i < MPC_HORIZON; i++) {
        mpc_param.Q_bar.block(i * Q.rows(), i * Q.cols(), Q.rows(), Q.cols()) = Q;
        mpc_param.R_bar.block(i * R.rows(), i * R.cols(), R.rows(), R.cols()) = R;
        if (i == 0)
            mpc_param.R_con_bar.block(0, 0, R_con.rows(), R_con.cols()) = R_con;
        else if (i == MPC_HORIZON - 1) {
            mpc_param.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = R_con;
            mpc_param.R_con_bar.block(i * R_con.rows(), (i - 1) * R_con.cols(), R_con.rows(), R_con.cols()) =
                -2 * R_con;
        } else {
            mpc_param.R_con_bar.block(i * R_con.rows(), i * R_con.cols(), R_con.rows(), R_con.cols()) = 2 * R_con;
            mpc_param.R_con_bar.block(i * R_con.rows(), (i - 1) * R_con.cols(), R_con.rows(), R_con.cols()) =
                -2 * R_con;
        }
    }
    mpc_param.Q_bar.block((MPC_HORIZON - 1) * Q.rows(), (MPC_HORIZON - 1) * Q.cols(), F.rows(), F.cols()) = F;

    mpc_param.H.resize(mpc_param.C.rows(), mpc_param.C.cols());
    mpc_param.H = mpc_param.C.transpose() * mpc_param.Q_bar * mpc_param.C + mpc_param.R_bar + mpc_param.R_con_bar;
}

/**
 * @brief 设置安全飞行走廊。参考给定路径给出
 *
 * @param planes 飞行走廊的超平面
 * @param step 步数
 */
void MPC::setSFC(const std::vector<Eigen::Vector3d>& path, double distance) {
    size_t path_size = path.size();
    // 如果给定的轨迹长度大于设定的MPC HORIZON，限制到HORIZON长度
    if (path_size > MPC_HORIZON) {
        path_size = MPC_HORIZON;
    }

    // 限制矩阵大小
    mpc_param_.A_sfc.resize(3 * path_size, mpc_param_.A_p.cols());
    mpc_param_.A_sfc_low.resize(3 * path_size);
    mpc_param_.A_sfc_upp.resize(3 * path_size);

    // 将给定的轨迹填充到VectorXd中
    Eigen::Vector3d vec_dis  = distance * Eigen::Vector3d::Ones();
    Eigen::VectorXd low_path = Eigen::VectorXd::Zero(3 * path_size);
    Eigen::VectorXd upp_path = Eigen::VectorXd::Zero(3 * path_size);
    for (Eigen::Index i = 0; i < path_size; ++i) {
        low_path.segment(3 * i, 3) = path.at(i) - vec_dis;
        low_path(3 * i + 2)        = path.at(i).z();
        upp_path.segment(3 * i, 3) = path.at(i) + vec_dis;
        upp_path(3 * i + 2)        = path.at(i).z();
    }
    // 更新数据
    mpc_param_.A_sfc     = mpc_param_.A_p.block(0, 0, 3 * path_size, mpc_param_.A_p.cols());
    mpc_param_.A_sfc_low = low_path;
    mpc_param_.A_sfc_upp = upp_path;
}

/**
 * @brief 设置当前的状态值：位置、速度和加速度
 *
 * @param p0 当前的位置
 * @param v0 当前的速度
 * @param a0 当前的加速度
 */
void MPC::setStatus(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0) {
    // 限制速度和加速度的取值
    limitStatus(v0, a0);

    // 设置当前的状态值
    X_0_.segment(0, p0.rows())                     = p0;
    X_0_.segment(p0.rows(), v0.rows())             = v0;
    X_0_.segment(p0.rows() + v0.rows(), a0.rows()) = a0;
}

/**
 * @brief 限制给定的速度和加速度数值，在最大值和最小值中间
 */
void MPC::limitStatus(Eigen::Vector3d& v0, Eigen::Vector3d& a0) {
    for (int i = 0; i < 3; ++i) {
        v0(i) = std::clamp(v0(i), v_min_(i), v_max_(i));
        a0(i) = std::clamp(a0(i), a_min_(i), a_max_(i));
    }
}

/**
 * @brief 更新边界条件
 */
void MPC::updateBound(MpcParameter& mpc_param, const Eigen::VectorXd& x_0) {
    if (x_0.rows() != mpc_param.M_p.cols() || x_0.rows() != mpc_param.M_v.cols()
        || x_0.rows() != mpc_param.M_a.cols()) {
        ROS_ERROR("[MPC]: Update bound error!");
        return;
    }

    /*
        模型如下：
        x(t) = Mx(0) + Cu，
        u = [u(0), u(1), ..., u(t-1)]^T
    */

    // 更新初始状态得到的数值
    mpc_param.B_p = mpc_param.M_p * x_0;
    mpc_param.B_v = mpc_param.M_v * x_0;
    mpc_param.B_a = mpc_param.M_a * x_0;
    // 更新u
    Eigen::Index u_low_rows = mpc_param.u_low.rows(), u_upp_rows = mpc_param.u_upp.rows();
    mpc_param.A_sys_low.segment(0, u_low_rows) = mpc_param.u_low;
    mpc_param.A_sys_upp.segment(0, u_upp_rows) = mpc_param.u_upp;
    // 更新v
    Eigen::Index v_low_rows = mpc_param.v_low.rows(), v_upp_rows = mpc_param.v_upp.rows();
    mpc_param.A_sys_low.segment(u_low_rows, v_low_rows) = mpc_param.v_low - mpc_param.B_v;
    mpc_param.A_sys_upp.segment(u_upp_rows, v_upp_rows) = mpc_param.v_upp - mpc_param.B_v;
    // 更新a
    Eigen::Index a_low_rows = mpc_param.a_low.rows(), a_upp_rows = mpc_param.a_upp.rows();
    mpc_param.A_sys_low.segment(u_low_rows + v_low_rows, a_low_rows) = mpc_param.a_low - mpc_param.B_a;
    mpc_param.A_sys_upp.segment(u_upp_rows + v_upp_rows, a_upp_rows) = mpc_param.a_upp - mpc_param.B_a;
}

/**
 * @brief 获得最优控制命令
 *
 * @param u 对应的控制命令
 * @param segment 指定的段数
 */
void MPC::getOptimCmd(Eigen::Vector3d& u, int seg_id) {
    // 如果给定段数大于MPC限制，返回最后一段控制
    if (seg_id >= MPC_HORIZON) {
        seg_id = MPC_HORIZON - 1;
    }

    // 三轴最优控制
    u.x() = mpc_param_.u_optimal(3 * seg_id + 0, 0);
    u.y() = mpc_param_.u_optimal(3 * seg_id + 1, 0);
    u.z() = mpc_param_.u_optimal(3 * seg_id + 2, 0);
}
