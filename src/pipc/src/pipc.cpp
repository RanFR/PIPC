#include "pipc/pipc.h"

/**
 * @brief 初始化PIPC类，引入Ros::NodeHandle
 */
PIPC::PIPC(ros::NodeHandle& nh) {
    /* 加载Ros参数 */
    // 程序相关设置参数
    double freq;
    nh.param("simulation", simu_flag_, true);
    nh.param("frequency", freq, 100.0);

    // 安全距离
    nh.param("safe_dis", safe_dis_, 0.1);

    // 无人机控制相关参数
    nh.param("thrust_limit", thrust_limit_, 0.5);
    nh.param("hover_esti", hover_esti_flag_, true);
    nh.param("hover_perc", hover_perc_, 0.23);
    nh.param("yaw_ctrl_flag", yaw_ctrl_flag_, false);
    nh.param("yaw_gain", yaw_gain_, 0.1);

    // 搜索用地图相关参数
    double map_x_size {10.0}, map_y_size {10.0}, map_z_size {5.0};
    nh.param("map/resolution", resolution_, 0.1);
    nh.param("map/map_x_size", map_x_size, 10.0);
    nh.param("map/map_y_size", map_y_size, 10.0);
    nh.param("map/map_z_size", map_z_size, 5.0);

    // 根据给定的参数设置地图大小
    map_low_ << -map_x_size / 2.0, -map_y_size / 2.0, 0.0;
    map_upp_ << map_x_size / 2.0, map_y_size / 2.0, map_z_size;

    // 初始化里程计和目标参赛参数
    goal_pos_ << 0.0, 0.0, 1.0;
    odom_pos_ << 0.0, 0.0, 1.0;
    Gravity_ << 0, 0, 9.81;
    odom_vel_.setZero();
    odom_acc_.setZero();
    imu_acc_.setZero();

    // 初始化dwa路径

    for (int i = 0; i < 3; ++i) {
        double tmp_time {ros::Time::now().toSec()};
        dwa_path_.push_back(odom_pos_);
        dwa_time_path_.push_back({odom_pos_, tmp_time});
    }

    // 设置仿真环境或者实际飞行环境，决定开始的推力和程序模式
    if (simu_flag_) {
        thrust_ = 0.7;
        mode_   = Command;
    } else {
        thrust_ = hover_perc_;
        mode_   = Manual;
        ROS_INFO("\033[32m[PIPC] Switch to Manual.\033[0m");
    }
    thr2acc_ = 9.81 / thrust_;

    // 实例化障碍物预测
    obs_pred_ptr_ = std::make_shared<ObstaclesPrediction>();
    obs_pred_ptr_->init(nh);

    // 实例化DWA方法
    dwa_ptr_ = std::make_unique<DWA>();
    dwa_ptr_->init(nh);
    dwa_ptr_->setMapParameters(resolution_, map_x_size, map_y_size, map_z_size);
    dwa_ptr_->setObstaclePrediction(obs_pred_ptr_);

    // 实例化mpc求解器
    mpc_ = std::make_unique<MPC>();
    mpc_->init(nh);

    // Ros发布消息话题
    dwa_path_pub_ = nh.advertise<nav_msgs::Path>("dwa_path", 1);
    mpc_path_pub_ = nh.advertise<nav_msgs::Path>("mpc_path", 1);
    goal_pub_     = nh.advertise<geometry_msgs::PoseStamped>("goal_point", 1);
    cmd_pub_      = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 1);
    px4ctrl_pub_  = nh.advertise<mavros_msgs::AttitudeTarget>("px4ctrl", 1);

    // Ros订阅话题
    // local pc用于检测碰撞
    local_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("local_pc", 100, &PIPC::rcvLocalPcCallback, this);
    // static用于生成dwa路径
    static_pc_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("static_pc", 10, &PIPC::rcvStaticPcCallback, this);
    odom_sub_      = nh.subscribe<nav_msgs::Odometry>("odom", 1, &PIPC::rcvOdomCallback, this);
    goal_sub_      = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &PIPC::rcvGoalCallback, this);
    imu_sub_       = nh.subscribe<sensor_msgs::Imu>("imu", 10, &PIPC::rcvIMUCallback, this);
    rc_sub_        = nh.subscribe<mavros_msgs::RCIn>("rc_in", 1, &PIPC::rcvRCInCallback, this);

    // Ros定时器
    timer_ = nh.createTimer(ros::Duration(1.0 / freq), &PIPC::execTimerCallback, this);
    // replan_timer_ = nh.createTimer(ros::Duration(0.1), &PIPC::execReplanTimer, this);
}

/**
 * @brief 发布kino astar轨迹数据
 *
 * @param nodes 存储了kino astar轨迹点的vector
 * @param scale 发布的轨迹的粗细程度
 */
void PIPC::pubDWAPath(const std::vector<Eigen::Vector3d>& path) {
    nav_msgs::Path msg;

    // Header
    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();

    // 路径
    for (const Eigen::Vector3d& pt : path) {
        geometry_msgs::PoseStamped pos;
        pos.header.frame_id    = "world";
        pos.header.stamp       = ros::Time::now();
        pos.pose.position.x    = pt.x();
        pos.pose.position.y    = pt.y();
        pos.pose.position.z    = pt.z();
        pos.pose.orientation.w = 1.0;
        msg.poses.push_back(pos);
    }

    dwa_path_pub_.publish(msg);
}

/**
 * @brief 根据输入的期望位置、速度、加速度和加加速度发布对应控制命令
 *
 * @param p_r 期望的位置
 * @param v_r 期望的速度
 * @param a_r 期望的加速度
 * @param j_r 期望的加加速度
 */
void PIPC::pubCmd(Eigen::Vector3d p_r, Eigen::Vector3d v_r, Eigen::Vector3d a_r, Eigen::Vector3d j_r) {
    quadrotor_msgs::PositionCommand msg;
    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    msg.position.x      = p_r.x();
    msg.position.y      = p_r.y();
    msg.position.z      = p_r.z();
    msg.velocity.x      = v_r.x();
    msg.velocity.y      = v_r.y();
    msg.velocity.z      = v_r.z();
    msg.acceleration.x  = a_r.x();
    msg.acceleration.y  = a_r.y();
    msg.acceleration.z  = a_r.z();
    msg.jerk.x          = j_r.x();
    msg.jerk.y          = j_r.y();
    msg.jerk.z          = j_r.z();

    // 启动yaw角控制
    if (yaw_ctrl_flag_) {
        double yaw_error = yaw_r_ - yaw_;
        // 限制yaw范围在-pi到pi之间
        if (yaw_error > M_PI)
            yaw_error -= M_PI * 2;
        if (yaw_error < -M_PI)
            yaw_error += M_PI * 2;
        msg.yaw     = yaw_ + yaw_error * 0.1;
        msg.yaw_dot = 0;
    } else {
        // 不控制yaw，直接设置为0
        msg.yaw     = 0;
        msg.yaw_dot = 0;
    }
    cmd_pub_.publish(msg);
}

/**
 * @brief 发布mpc轨迹
 *
 * @param pt 给定的mpc轨迹
 */
void PIPC::pubMPCPath(const std::vector<Eigen::Vector3d>& path) {
    nav_msgs::Path msg;
    msg.header.frame_id = "world";
    msg.header.stamp    = ros::Time::now();
    for (const Eigen::Vector3d& pt : path) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = pt.z();
        msg.poses.push_back(pose);
    }

    mpc_path_pub_.publish(msg);
}

/**
 * @brief 发布高度控制的命令
 *
 * @param q 无人机四元数信息
 * @param thrust 无人机推力信息
 * @param stamp 时间戳信息
 */
void PIPC::pubAttitudeCtrl(const Eigen::Quaterniond& q, const double thrust, const ros::Time& stamp) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = std::string("FCU");
    msg.type_mask       = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE
                    | mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    // 四元数
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    msg.thrust        = thrust;

    // 限制推力在0.1到0.9之间
    if (msg.thrust < 0.1) {
        msg.thrust = 0.1;
    }
    if (msg.thrust > 0.9) {
        msg.thrust = 0.9;
    }
    // 手动控制，设置推力在0.05
    if (mode_ == Manual) {
        msg.thrust = 0.05;
    }
    // 不是仿真，同时推力大于限制，设置推力为限制值
    if (!simu_flag_ && msg.thrust > thrust_limit_) {
        msg.thrust = thrust_limit_;
    }
    // 发布
    px4ctrl_pub_.publish(msg);
}

/**
 * @brief 发布角速度控制命令
 *
 * @param rate 角速度
 * @param thrust 推力
 * @param stamp Ros下的时间戳
 */
void PIPC::pubBodyrateCtrl(const Eigen::Vector3d& rate, const double thrust, const ros::Time& stamp) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = std::string("FCU");
    msg.type_mask       = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.body_rate.x     = rate.x();
    msg.body_rate.y     = rate.y();
    msg.body_rate.z     = rate.z();
    msg.thrust          = thrust;

    // 限制推力在0.08到0.9之间
    if (msg.thrust < 0.08) {
        msg.thrust = 0.08;
    }
    if (msg.thrust > 0.9) {
        msg.thrust = 0.9;
    }
    // 手动控制，设置推力默认为0.05
    if (mode_ == Manual) {
        msg.thrust = 0.05;
    }
    // 非仿真，同时推力超过限制，设置为限制值
    if (!simu_flag_ && msg.thrust > thrust_limit_) {
        msg.thrust = thrust_limit_;
    }
    // 发布
    px4ctrl_pub_.publish(msg);
}

/**
 * @brief 根据加速度信息计算推力
 */
void PIPC::computeThrust(const Eigen::Vector3d& acc) {
    const Eigen::Vector3d zB           = odom_quat_ * Eigen::Vector3d::UnitZ();
    double                des_acc_norm = acc.dot(zB);
    thrust_                            = des_acc_norm / thr2acc_;
}

/**
 * @brief 合并控制信息
 */
void PIPC::convertCommand(Eigen::Vector3d acc, Eigen::Vector3d jerk) {
    Eigen::Vector3d xB, yB, zB, xC;
    if (yaw_ctrl_flag_) {
        double yaw_error = yaw_r_ - yaw_;
        if (yaw_error > M_PI) {
            yaw_error -= M_PI * 2;
        }
        if (yaw_error < -M_PI) {
            yaw_error += M_PI * 2;
        }
        yaw_dot_r_ = yaw_error * yaw_gain_;
    } else {
        yaw_dot_r_ = (0.0 - yaw_) * yaw_gain_;
    }
    xC << std::cos(yaw_), std::sin(yaw_), 0;

    zB = acc.normalized();
    yB = (zB.cross(xC)).normalized();
    xB = yB.cross(zB);
    Eigen::Matrix3d R;
    R << xB, yB, zB;
    u_quat_ = R;

    Eigen::Vector3d hw = (jerk - (zB.dot(jerk) * zB)) / acc.norm();
    rate_.x()          = -hw.dot(yB);
    rate_.y()          = hw.dot(xB);
    rate_.z()          = yaw_dot_r_ * zB.dot(Eigen::Vector3d(0, 0, 1));
}

/**
 * @brief 估计推力模型
 */
bool PIPC::estimateThrustModel(const Eigen::Vector3d& est_a) {
    // 不需要估计悬停
    if (!hover_esti_flag_) {
        // 直接根据悬停油门估计推力
        thr2acc_ = 9.81 / hover_perc_;
        return true;
    }

    // 当前模式非程序控制
    if (mode_ != Command) {
        P_ = 100.0;
        // 根据已有的悬停率估算推力
        thr2acc_ = 9.81 / hover_perc_;
        return true;
    }

    ros::Time t_now = ros::Time::now();
    // 用于估算的时间和推力序列
    if (timed_thrust_.size() == 0) {
        return false;
    }

    std::pair<ros::Time, double> t_t = timed_thrust_.front();

    // 序列存在，开始估计
    while (timed_thrust_.size() >= 1) {
        // 当前时间与给定的初始时间差
        double delta_t = (t_now - t_t.first).toSec();
        // 差值大于1秒，弹出初始时间，重新循环
        if (delta_t > 1.0) {
            timed_thrust_.pop();
            continue;
        }
        // 差值小于0.035秒，放弃估算
        if (delta_t < 0.035) {
            return false;
        }

        /* 具有消失记忆的递归最小二乘算法 */
        /* Recursive least squares algorithm with vanishing memory */
        double thr = t_t.second;
        timed_thrust_.pop();
        /* Model: est_a(2) = thr2acc * thr */
        double R                = 0.3;  // using Kalman filter
        double K                = P_ / (P_ + R);
        thr2acc_                = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_                      = (1 - K * thr) * P_;
        double hover_percentage = 9.81 / thr2acc_;
        if (hover_percentage > 0.8 || hover_percentage < 0.1) {
            // 限制最大和最小的推力
            thr2acc_ = hover_percentage > 0.8 ? 9.81 / 0.8 : thr2acc_;
            thr2acc_ = hover_percentage < 0.1 ? 9.81 / 0.1 : thr2acc_;
        }
        return true;
    }
    return false;
}

/**
 * @brief 接受里程计的回调函数
 */
void PIPC::rcvOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    // 互斥锁
    odom_mutex_.lock();

    // 里程计时间、位置、速度和四元数
    odom_time_ = msg->header.stamp;
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    odom_quat_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);
    // 根据里程计数据计算加速度
    odom_acc_ = odom_quat_ * Eigen::Vector3d(0, 0, 1) * (thrust_ * thr2acc_) - Gravity_;

    // 根据四元数获得yaw角
    const geometry_msgs::Quaternion& quat_msg = msg->pose.pose.orientation;
    tf2::Quaternion                  quat;
    tf2::fromMsg(quat_msg, quat);
    double roll, pitch;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw_);

    // 接收到odom的标志
    has_odom_flag_ = true;

    // 解除互斥锁
    odom_mutex_.unlock();
}

/**
 * @brief 接收给定的目标数据
 */
void PIPC::rcvGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    // 互斥锁
    goal_mutex_.lock();

    // 给定的新目标
    // TEST:暂时设定目标点为(10, 0, 1)
    // Eigen::Vector3d new_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // Eigen::Vector3d new_goal(msg->pose.position.x, msg->pose.position.y, 1.0);
    Eigen::Vector3d new_goal(28.0, 0.0, 1.0);

    // 如果两次给定的目标不相同，同时当前为程序控制
    if (goal_pos_ != new_goal && mode_ == Command) {
        // 设置新的目标
        // goal_pos_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        // goal_pos_ << msg->pose.position.x, msg->pose.position.y, 1.0;
        goal_pos_ = new_goal;

        // 限制目标点的z轴不超过地图高度，同时不低于0.5米
        if (goal_pos_.z() > map_upp_.z() - 0.5) {
            goal_pos_.z() = map_upp_.z() - 0.5;
        }
        if (goal_pos_.z() < 0.5) {
            goal_pos_.z() = 0.5;
        }
        // 新目标标志为true
        new_goal_flag_ = true;
    }

    // 解除互斥锁
    goal_mutex_.unlock();
}

/**
 * @brief 接收IMU数据
 */
void PIPC::rcvIMUCallback(const sensor_msgs::ImuConstPtr& msg) {
    // 互斥锁
    imu_mutex_.lock();

    // 读取机身坐标下的IMU数据
    imu_acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;  // body frame

    // 根据机身四元数信息计算旋转矩阵
    Eigen::Matrix3d Rotate = odom_quat_.toRotationMatrix().inverse();
    // 将机身坐标下的IMU转化为世界坐标系下的IMU数据
    imu_acc_ = Rotate * imu_acc_;

    // 解除互斥锁
    imu_mutex_.unlock();
}

/**
 * @brief
 * 局部点云的回调函数。（1）接收点云信息，降采样；（2）设置初始路径搜索障碍信息与中心点；（3）检查路径是否与更新后地图碰撞，设置重规划标志。
 */
void PIPC::rcvLocalPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    // 互斥锁
    local_pc_mutex_.lock();

    // 将ros点云数据转化为pcl格式
    pcl::PointCloud<pcl::PointXYZ> local_cloud;
    pcl::fromROSMsg(*msg, local_cloud);

    // 使用CropBox方法进行点云剔除
    pcl::CropBox<pcl::PointXYZ> cb;
    // 以记录的里程计的点作为中心点，根据地图的大小设置剔除的点云范围
    cb.setMin(Eigen::Vector4f(odom_pos_.x() - (map_upp_.x() - 0.5), odom_pos_.y() - (map_upp_.y() - 0.5), 0.2, 1.0));
    cb.setMax(
        Eigen::Vector4f(odom_pos_.x() + (map_upp_.x() - 0.5), odom_pos_.y() + (map_upp_.y() - 0.5), map_upp_.z(), 1.0));
    // 设置输入点云为之前记录的静态点云
    cb.setInputCloud(local_cloud.makeShared());
    // 进行点云剔除
    cb.filter(local_cloud);

    // 将输入的静态点云转化为栅格点云地图降采样
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    vf.setInputCloud(local_cloud.makeShared());
    vf.filter(local_cloud);

    if (local_cloud.size() == 0 && !has_rcv_pc_msg_) {
        local_pc_mutex_.unlock();
        return;
    }

    // 同步DWA地图中心
    dwa_ptr_->setMapCenter(Eigen::Vector3d(odom_pos_.x(), odom_pos_.y(), 0.0));

    // 设置DWA使用的地图
    dwa_ptr_->setMap(local_cloud);

    // 增加对路径是否与更新后障碍碰撞的部分，设置重规划标志。
    if (checkPathFree() && !replan_flag_) {
        replan_flag_ = true;
    }

    // 接收到点云数据
    has_rcv_pc_msg_ = true;

    // 解除互斥锁
    local_pc_mutex_.unlock();
}

void PIPC::rcvStaticPcCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    if (!has_rcv_pc_msg_) {
        return;
    }

    // 互斥锁
    static_pc_mutex_.lock();

    // 根据输入的点云信息获得PCL点云地图，并得到对应的指针
    pcl::fromROSMsg(*msg, static_cloud_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_ptr {static_cloud_.makeShared()};

    // 使用CropBox方法进行点云剔除
    pcl::CropBox<pcl::PointXYZ> cb;  // CropBox filter (delete unuseful points)
    // 以记录的里程计的点作为中心点，根据地图的大小设置剔除的点云范围
    cb.setMin(Eigen::Vector4f(odom_pos_.x() - (map_upp_.x() - 0.5), odom_pos_.y() - (map_upp_.y() - 0.5), 0.2, 1.0));
    cb.setMax(
        Eigen::Vector4f(odom_pos_.x() + (map_upp_.x() - 0.5), odom_pos_.y() + (map_upp_.y() - 0.5), map_upp_.z(), 1.0));
    // 设置输入点云为之前记录的静态点云
    cb.setInputCloud(static_cloud_ptr);
    // 进行点云剔除
    cb.filter(static_cloud_);
    // 将输入的静态点云转化为栅格点云地图
    pcl::VoxelGrid<pcl::PointXYZ> vf;
    vf.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    vf.setInputCloud(static_cloud_ptr);
    vf.filter(static_cloud_);

    if (static_cloud_.size() == 0) {
        static_cloud_.push_back(pcl::PointXYZ(map_low_.x(), map_low_.y(), map_low_.z()));
        static_pc_mutex_.unlock();
        return;
    }

    // 设置DWA所使用的静态地图信息
    dwa_ptr_->setStaticMap(static_cloud_);

    // 解除互斥锁
    static_pc_mutex_.unlock();
}

void PIPC::rcvRCInCallback(const mavros_msgs::RCInConstPtr& msg) {
    // 线程锁
    rc_mutex_.lock();

    if (simu_flag_) {
        // 仿真模式，默认设置为成程序控制
        mode_ = Command;
    } else {
        // 遥控器控制，默认为手动飞行模式，需要手动切换
        if (msg->channels[5] < 1250 && mode_ != Manual) {
            mode_ = Manual;
            ROS_INFO("\033[32m[PIPC] Switch to Manual.\033[0m");
        }
        // 悬停模式、降落模式和程序控制共用遥控器6通道
        if (msg->channels[5] > 1250) {
            // 降落模式
            if (msg->channels[7] < 1250 && mode_ != Land) {
                mode_ = Land;
                goal_pos_ << odom_pos_.x(), odom_pos_.y(), 0.3;
                ROS_INFO("\033[32m[PIPC] Switch to Land.\033[0");
            }
            // 悬停模式
            if (msg->channels[7] > 1250 && msg->channels[7] < 1750 && mode_ != Hover) {
                mode_ = Hover;
                goal_pos_ << odom_pos_.x(), odom_pos_.y(), 1.0;
                ROS_INFO("\033[32m[PIPC] Switch to Hover. Hover position is %f %f %f\033[0,m",
                         goal_pos_.x(),
                         goal_pos_.y(),
                         goal_pos_.z());
            }
            // 程控模式
            if (msg->channels[7] > 1750 && mode_ != Command) {
                mode_ = Command;
                ROS_INFO("\033[32m[PIPC] Switch to Command.\033[0m");
            }
        }
    }

    rc_mutex_.unlock();
}

/**
 * @brief 定时器回调函数
 */
void PIPC::execTimerCallback(const ros::TimerEvent&) {
    // 检查是否接收到里程计信息
    if (!has_odom_flag_) {
        return;
    }

    if (!has_rcv_pc_msg_) {
        return;
    }

    // 检查当前模式是否为手动控制
    if (mode_ == Manual) {
        // 如果为手动控制，则发布角速度控制信号
        pubBodyrateCtrl(Eigen::Vector3d::Zero(), 0.05, ros::Time::now());
        return;
    }

    // 互斥锁
    timer_mutex_.lock();

    // 如果模式为程序控制模式
    if (mode_ == Command) {
        // 接收到新的目标，启用路径重规划
        if (new_goal_flag_ || replan_flag_) {
            new_goal_flag_ = false;
            replan_flag_   = false;
            double time1 {ros::Time::now().toSec()};
            replanPath();
            std::cerr << "duation: " << ros::Time::now().toSec() - time1 << std::endl;
        }

        // // 如果odom与goal的距离小于设定的阈值，不接收follow path控制，自行前往终点
        // if ((odom_pos_ - goal_pos_).norm() < 4.0) {
        //     if ((odom_pos_ - goal_pos_).norm() < 0.2) {
        //         std::vector<Eigen::Vector3d> tmp_path;
        //         for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
        //             mpc_->setGoal(goal_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        //             tmp_path.push_back(goal_pos_);
        //         }
        //         follow_path_ = tmp_path;
        //     } else {
        //         std::vector<Eigen::Vector3d> tmp_path;
        //         Eigen::Vector3d              direction = goal_pos_ - odom_pos_;
        //         for (int i = 0; i < mpc_->MPC_HORIZON; ++i) {
        //             Eigen::Vector3d hover_pos {
        //                 odom_pos_ + static_cast<double>(i + 1) / static_cast<double>(mpc_->MPC_HORIZON) * direction};
        //             mpc_->setGoal(hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
        //             tmp_path.push_back(hover_pos);
        //         }
        //         follow_path_ = tmp_path;
        //     }
        //     reach_target_ = true;
        // }

        static double last_time {ros::Time::now().toSec()};
        double        current_time {ros::Time::now().toSec()};
        if (current_time - last_time > 0.5 || dwa_index_ >= follow_path_.size() * 3 / 4) {
            // if (dwa_index_ >= follow_path_.size() * 3 / 4) {
            // if (dwa_index_ >= 24 || dwa_index_ >= follow_path_.size() - 1) {
            replan_flag_ = true;
            last_time    = current_time;
        }

        static Eigen::Vector3d tmp_hover_pos;
        static bool            has_been_set {false};
        /* 设置SFC */
        if (follow_path_.size() > 1) {
            // 搜索路径非空，遍历follow path，找到距离里程计最近的点，默认为当前的起点
            double min_dis = 1e3;
            for (size_t i = 0; i < follow_path_.size(); ++i) {
                double dis = (odom_pos_ - follow_path_[i]).norm();
                if (dis < min_dis) {
                    min_dis    = dis;
                    dwa_index_ = i;
                }
            }
            // ++dwa_index_;

            // if (dwa_index_ != 0) {
            //     // std::cerr << "dwa idx: " << dwa_index_;
            //     // 获取当前路径最后的状态
            //     for (size_t i = 1; i < dwa_tuple_path_.size(); ++i) {
            //         // std::cerr << ", i: " << i;
            //         auto [start_p, start_v, start_a] = dwa_tuple_path_[dwa_tuple_path_.size() - i];
            //         // std::cerr << ", get";
            //         // std::cerr << ", pos: " << start_p.transpose() << ", vel: " << start_v.transpose()
            //         //           << ", acc: " << start_a.transpose();
            //         std::vector<std::vector<DWANodePtr>> tree = dwa_ptr_->search(start_p,
            //                                                                      start_v,
            //                                                                      start_a,
            //                                                                      goal_pos_,
            //                                                                      Eigen::Vector3d::Zero(),
            //                                                                      ros::Time::now().toSec(),
            //                                                                      i);
            //         // std::cerr << ", tree size: " << tree.size();

            //         // 如果搜索的路径小于已经行驶过的路径，尝试回退重新搜索
            //         if (tree.size() == 1) {
            //             continue;
            //         } else {
            //             // std::cerr << ", else:1";
            //             std::vector<Eigen::Vector3d>                     additional_path =
            //             dwa_ptr_->getBestPath(tree); std::vector<std::tuple<Eigen::Vector3d, double>>
            //             additional_time_path =
            //                 dwa_ptr_->getBestTimePath(tree);
            //             std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>>
            //                 additional_tuple_path = dwa_ptr_->getBestTuplePath(tree);
            //             // std::cerr << "2";

            //             //插入到现有路径中
            //             dwa_path_.insert(dwa_path_.end(), additional_path.begin(), additional_path.end());
            //             dwa_time_path_.insert(dwa_time_path_.end(),
            //                                   additional_time_path.begin(),
            //                                   additional_time_path.end());
            //             dwa_tuple_path_.insert(dwa_tuple_path_.end(),
            //                                    additional_tuple_path.begin(),
            //                                    additional_tuple_path.end());

            //             // std::cerr << "3";

            //             // 删除已经行驶过的路径
            //             dwa_path_.erase(dwa_path_.begin(), dwa_path_.begin() + dwa_index_);
            //             dwa_time_path_.erase(dwa_time_path_.begin(), dwa_time_path_.begin() + dwa_index_);
            //             dwa_tuple_path_.erase(dwa_tuple_path_.begin(), dwa_tuple_path_.begin() + dwa_index_);

            //             // std::cerr << "4";

            //             follow_path_ = dwa_path_;
            //             dwa_index_   = 0;

            //             pubDWAPath(dwa_path_);

            //             // std::cerr << "5.";

            //             break;
            //         }
            //     }

            //     if (dwa_path_.size() < 32) {
            //         replan_flag_ = true;
            //     }
            // }

            // 根据搜索到的起点，对搜索的路径进行切割，用以设置SFC
            std::vector<Eigen::Vector3d> sfc_path(follow_path_.begin() + dwa_index_, follow_path_.end());
            mpc_->setSFC(sfc_path, 0.1);

            /* 设置MPC目标点 */
            mpc_goals_.clear();
            for (int i = 0; i < mpc_->MPC_HORIZON; ++i) {
                int index = dwa_index_ + i;
                if (index >= follow_path_.size()) {
                    index = follow_path_.size() - 1;
                }

                // 融合DWA搜索获得的速度和加速度
                auto [pos, vel, acc] = dwa_tuple_path_[index];

                // 设置优化目标
                mpc_->setGoal(pos, vel, acc, i);
                // mpc_->setGoal(pos, vel, Eigen::Vector3d::Zero(), i);

                mpc_goals_.push_back(pos);
            }

            // Yaw控制
            if (dwa_index_ < follow_path_.size() - 8) {
                Eigen::Vector3d back_pos = follow_path_.back();
                yaw_r_                   = std::atan2(back_pos.y() - odom_pos_.y(), back_pos.x() - odom_pos_.x());
                if ((follow_path_.back() - odom_pos_).norm() <= 0.25) {
                    yaw_r_ = 0.0;
                }
            }

            has_been_set = false;
        } else {  // stay at initial position
            if (!has_been_set) {
                tmp_hover_pos << odom_pos_.x(), odom_pos_.y(), 1.0;
                has_been_set = true;
            }

            yaw_r_ = 0.0;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                mpc_->setGoal(tmp_hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
        }

        // 如果odom与goal的距离小于设定的阈值，不接收follow path控制，自行前往终点
        if ((odom_pos_ - goal_pos_).norm() < 4.0) {
            if ((odom_pos_ - goal_pos_).norm() < 0.2) {
                for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                    mpc_->setGoal(goal_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
                }
            } else {
                Eigen::Vector3d direction = goal_pos_ - odom_pos_;
                for (int i = 0; i < mpc_->MPC_HORIZON; ++i) {
                    Eigen::Vector3d hover_pos {
                        odom_pos_ + static_cast<double>(i + 1) / static_cast<double>(mpc_->MPC_HORIZON) * direction};
                    mpc_->setGoal(hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
                }
            }
            reach_target_ = true;
        }
    }

    // 悬停模式
    if (mode_ == Hover) {
        // 如果当前无人机距离悬停位置大于0.2米，采取分段设点形式
        if ((odom_pos_ - goal_pos_).norm() >= 0.2) {
            Eigen::Vector3d direction = goal_pos_ - odom_pos_;
            for (int i = 0; i < mpc_->MPC_HORIZON; ++i) {
                Eigen::Vector3d hover_pos {
                    odom_pos_ + static_cast<double>(i + 1) / static_cast<double>(mpc_->MPC_HORIZON) * direction};
                mpc_->setGoal(hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
        } else {
            // 直接设置所有跟踪点为goal_pos_
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                Eigen::Vector3d hover_pos {goal_pos_};
                mpc_->setGoal(hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
        }
    }

    // 降落模式
    if (mode_ == Land) {
        // 如果当前无人机距离降落高度大于0.2米，采取分段设点形式
        if (std::abs(odom_pos_.z() - goal_pos_.z()) >= 0.2) {
            Eigen::Vector3d direction = goal_pos_ - odom_pos_;
            for (int i = 0; i < mpc_->MPC_HORIZON; ++i) {
                Eigen::Vector3d hover_pos {
                    odom_pos_ + static_cast<double>(i + 1) / static_cast<double>(mpc_->MPC_HORIZON) * direction};
                mpc_->setGoal(hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
        } else {
            // 直接设置所有跟踪点为goal_pos_
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                Eigen::Vector3d hover_pos {goal_pos_};
                mpc_->setGoal(hover_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), i);
            }
        }
    }

    /* 运行 */
    // 设置状态
    mpc_->setStatus(odom_pos_, odom_vel_, odom_acc_);
    // 运行MPC
    bool success_flag = mpc_->run();

    Eigen::Vector3d u_optimal, p_optimal, v_optimal, a_optimal, u_predict;
    Eigen::MatrixXd A1, B1;
    Eigen::VectorXd x_optimal = mpc_->X_0_;

    // 如果优化成功
    if (success_flag) {
        last_mpc_time_ = ros::Time::now();

        // 更新mpc控制索引
        mpc_ctrl_index_ = 0;

        // 获得当前最优状态
        mpc_->getOptimCmd(u_optimal, 0);
        mpc_->generateSystemModel(A1, B1, mpc_->MPC_STEP);
        x_optimal = A1 * x_optimal + B1 * u_optimal;

        p_optimal << x_optimal(0), x_optimal(1), x_optimal(2);
        v_optimal << x_optimal(3), x_optimal(4), x_optimal(5);
        a_optimal << x_optimal(6), x_optimal(7), x_optimal(8);

        // 发布控制命令
        // mpc_->getOptimCmd(u_optimal, 1);
        pubCmd(p_optimal, v_optimal, a_optimal, u_optimal);

        // 计算未来HORIZON下的最优状态
        std::vector<Eigen::Vector3d> path;
        x_optimal = mpc_->X_0_;
        for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
            mpc_->getOptimCmd(u_predict, i);
            x_optimal = A1 * x_optimal + B1 * u_predict;
            path.push_back(Eigen::Vector3d(x_optimal(0, 0), x_optimal(1, 0), x_optimal(2, 0)));
        }
        pubMPCPath(path);
    } else {
        // 优化失败，复用之前的结果
        // FIXME:有待商榷
        double delta_t = (ros::Time::now() - last_mpc_time_).toSec();
        if (delta_t >= mpc_->MPC_STEP) {
            mpc_ctrl_index_ += delta_t / mpc_->MPC_STEP;
            last_mpc_time_ = ros::Time::now();
        }
        mpc_->getOptimCmd(u_optimal, mpc_ctrl_index_);
        mpc_->generateSystemModel(A1, B1, mpc_->MPC_STEP);

        x_optimal = A1 * mpc_->X_0_ + B1 * u_optimal;
        p_optimal << x_optimal(0, 0), x_optimal(1, 0), x_optimal(2, 0);
        v_optimal << x_optimal(3, 0), x_optimal(4, 0), x_optimal(5, 0);
        a_optimal << x_optimal(6, 0), x_optimal(7, 0), x_optimal(8, 0);

        // 发布控制命令
        pubCmd(p_optimal, v_optimal, a_optimal, u_optimal);
    }

    ros::Time df_start = ros::Time::now();

    // 估算推力模型
    estimateThrustModel(imu_acc_);
    a_optimal += Gravity_;
    computeThrust(a_optimal);
    convertCommand(a_optimal, u_optimal);
    pubBodyrateCtrl(rate_, thrust_, ros::Time::now());

    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), thrust_));
    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }

    // 解除互斥锁
    timer_mutex_.unlock();
}

void PIPC::execReplanTimer(const ros::TimerEvent& event) {
    replan_timer_mutex_.lock();
    // 接收到新的目标，启用路径重规划
    if (new_goal_flag_ || replan_flag_) {
        new_goal_flag_ = false;
        replan_flag_   = false;
        double time1 {ros::Time::now().toSec()};
        replanPath();
        std::cerr << "duation: " << ros::Time::now().toSec() - time1 << std::endl;
    }
    replan_timer_mutex_.unlock();
}

/**
 * @brief 重规划路径
 *
 * @return 是否需要重新设置A星搜索中的中心点。true表示重新设置中心点，false表示不变化
 * @retval true/false
 */
void PIPC::replanPath() {
    // 重置相关参数
    dwa_path_.clear();  // dwa路径，用于rviz中显示dwa路径
    dwa_time_path_.clear();
    dwa_tuple_path_.clear();  // dwa路径以及相关状态，包含速度和加速度
    follow_path_.clear();     // 需要跟随的路径，用于mpc优化

    // 起点位置、速度、加速度，终点位置、速度
    Eigen::Vector3d start_p, start_v, start_a, end_p, end_v;
    // 根据里程计信息设置起点
    // 如果重规划次数大于10次，重新设置起点速度和加速度
    start_p = odom_pos_, start_v = odom_vel_, start_a = odom_acc_;
    if (replan_times_ > 10) {
        ROS_INFO("[PIPC] Reset velocity and acceleration.");
        start_v.setZero();
        start_a.setZero();
    }

    end_p          = goal_pos_;
    end_v          = Eigen::Vector3d::Zero();
    double delta_x = goal_pos_.x() - odom_pos_.x();
    double delta_y = goal_pos_.y() - odom_pos_.y();
    // 如果目标与里程计数值偏差大于地图范围，将局部目标点限制到地图边界上
    if (std::abs(delta_x) > map_upp_.x() || std::abs(delta_y) > map_upp_.y()) {
        if (std::abs(delta_x) > std::abs(delta_y)) {
            end_p.x() = odom_pos_.x() + (delta_x / std::abs(delta_x)) * (map_upp_.x() - resolution_);
            end_p.y() = odom_pos_.y() + ((map_upp_.x() - resolution_) / std::abs(delta_x)) * delta_y;
        } else {
            end_p.x() = odom_pos_.x() + ((map_upp_.y() - resolution_) / std::abs(delta_y)) * delta_x;
            end_p.y() = odom_pos_.y() + (delta_y / std::abs(delta_y)) * (map_upp_.y() - resolution_);
        }
    }

    ROS_INFO("End position: %f, %f, %f", end_p.x(), end_p.y(), end_p.z());

    // 使用DWA搜索最优路径
    double start_time = ros::Time::now().toSec();
    start_p.z() = 1.0, start_v.z() = 0.0, start_a.z() = 0.0;
    std::vector<std::vector<DWANodePtr>> tree = dwa_ptr_->search(start_p, start_v, start_a, end_p, end_v, start_time);
    // 获取DWA相关路径
    dwa_path_       = dwa_ptr_->getBestPath(tree);
    dwa_time_path_  = dwa_ptr_->getBestTimePath(tree);
    dwa_tuple_path_ = dwa_ptr_->getBestTuplePath(tree);

    if (dwa_path_.size() > 1) {
        // 说明此时路线存在，直接设置即可
        follow_path_  = dwa_path_;
        replan_times_ = 0;
        pubDWAPath(dwa_path_);
    } else {
        // size为0或者1
        ROS_INFO("\033[41;37m[PIPC] No path! Stay at current point! \033[0m");

        // 没有搜索到路径，设置下一步点为默认时间间隔后的当前点，以防止乱飞
        follow_path_ = {odom_pos_};

        // 尝试重规划
        replan_flag_ = true;

        ++replan_times_;
    }

    // 发布目标点
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id             = "world";
    msg.header.stamp                = ros::Time::now();
    Eigen::Vector3d follow_goal_pos = follow_path_.back();
    msg.pose.position.x             = follow_goal_pos.x();
    msg.pose.position.y             = follow_goal_pos.y();
    msg.pose.position.z             = follow_goal_pos.z();
    goal_pub_.publish(msg);
}

/**
 * @brief 检查无人机飞行路径是否可行
 *
 * @return true表示不可行，需要重规划，false表示该路径可以继续保留
 */
bool PIPC::checkPathFree() {
    // 增加判断，如果dwa路径点个数为0或者1（默认没有路情况下设置的里程计位置），则不执行后面的判断
    if (dwa_path_.size() < 2) {
        return true;
    }

    // 检查从当前飞行起点到末尾是否有效
    std::vector<Eigen::Vector3d> check_path;
    check_path.insert(check_path.begin(), dwa_path_.begin() + dwa_index_, dwa_path_.end());
    std::vector<std::tuple<Eigen::Vector3d, double>> check_time_path;
    check_time_path.insert(check_time_path.begin(), dwa_time_path_.begin() + dwa_index_, dwa_time_path_.end());

    bool replan {false};
    for (size_t i = 0; i < check_path.size(); ++i) {
        bool is_occupied = dwa_ptr_->isOccupied(check_path[i]);
        if (is_occupied) {
            return true;
        }

        auto [pos, time]    = check_time_path[i];
        double dynamic_cost = obs_pred_ptr_->calculateUncertainty(time, pos);
        if (dynamic_cost < -0.1) {
            return true;
        }
    }

    return false;
}
