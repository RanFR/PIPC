#include "dwa/dwa.h"

void DWA::init(ros::NodeHandle& nh) {
    // 初始化地图中心点为原点
    center_.setZero();

    // 更新搜索中的最大速度和加速度
    nh.param("path_search/max_vel", max_vel_, 1.0);
    nh.param("path_search/max_acc", max_acc_, 1.0);
    nh.param("path_search/max_jerk", max_jerk_, 1.0);

    // 限制搜索的范围（全局地图范围）
    nh.param("path_search/global_map_size_x_min", global_map_size_x_min_, -1.0);
    nh.param("path_search/global_map_size_x_max", global_map_size_x_max_, 10.0);
    nh.param("path_search/global_map_size_y_min", global_map_size_y_min_, -5.0);
    nh.param("path_search/global_map_size_y_max", global_map_size_y_max_, 5.0);
    nh.param("path_search/global_map_size_z_min", global_map_size_z_min_, 0.1);
    nh.param("path_search/global_map_size_z_max", global_map_size_z_max_, 3.0);

    nh.param("path_search/weight_heuristic_distance", weight_heuristic_distance_, 1.0);
    nh.param("path_search/weight_static_obstacle_distance", weight_static_obstacle_distance_, 1.0);
    nh.param("path_search/weight_reference_point_distance", weight_reference_point_distance_, 1.0);
    nh.param("path_search/weight_dynamic_obstacle_uncertainty", weight_dynamic_obstacle_uncertainty_, 1.0);
    nh.param("path_search/weight_future_position", weight_future_position_, 1.0);

    // 预先创建节点指针
    int map_ptr_vec_size = std::pow(MAX_TREE_DEPTH, 2) * MAX_LAYER_NODE_NUM / 2;
    // MAX_TREE_DEPTH * SAMPLE_JERK_NUMBER * SAMPLE_JERK_NUMBER * SAMPLE_JERK_Z_NUMBER * MAX_LAYER_NODE_NUM;
    dwa_node_ptr_vec_.resize(map_ptr_vec_size);
    for (DWANodePtr& ptr : dwa_node_ptr_vec_) {
        ptr = std::make_unique<DWANode>();
    }

    // 初始化Astar模块
    astar_ptr_      = std::make_unique<Astar>();
    next_astar_ptr_ = std::make_unique<Astar>();

    ROS_INFO("Size: (%f, %f), (%f, %f), (%f, %f)",
             global_map_size_x_min_,
             global_map_size_x_max_,
             global_map_size_y_min_,
             global_map_size_y_max_,
             global_map_size_z_min_,
             global_map_size_z_max_);
}

void DWA::reset() {
    for (DWANodePtr& ptr : dwa_node_ptr_vec_) {
        ptr->pos.setZero();
        ptr->vel.setZero();
        ptr->acc.setZero();
        ptr->time   = 0.0;
        ptr->score  = 0.0;
        ptr->cost   = 0.0;
        ptr->parent = nullptr;
    }
}

/**
 * @brief 设置地图中心点
 */
void DWA::setMapCenter(const Eigen::Vector3d& center) {
    center_ = center;

    astar_ptr_->setCenter(center);
    next_astar_ptr_->setCenter(center);
}

/**
 * @brief 设置地图参数，包括分辨率，xyz三轴宽度
 */
void DWA::setMapParameters(double resolution, double map_x_size, double map_y_size, double map_z_size) {
    // 分辨率
    resolution_ = resolution;

    // 地图大小
    low_bound_ << -map_x_size / 2.0, -map_y_size / 2.0, 0.0;
    upp_bound_ << map_x_size / 2.0, map_y_size / 2.0, map_z_size;

    // 更新地图索引大小
    map_size_ = ((upp_bound_ - low_bound_) / resolution_).cast<int>();

    // 更新astar地图参数
    Eigen::Vector3d astar_low_bound {-map_x_size, -map_y_size, 0.0};
    Eigen::Vector3d astar_upp_bound {map_x_size, map_y_size, map_z_size};
    astar_ptr_->initMap(0.25, low_bound_, upp_bound_);
    next_astar_ptr_->initMap(0.25, low_bound_, upp_bound_);
}

/**
 * @brief 设置点云地图
 */
void DWA::setMap(const pcl::PointCloud<pcl::PointXYZ>& pcl_map) {
    // 将pcl点云转化为KD树
    kd_tree_flann_mutex_.lock();
    kd_tree_flann_.setInputCloud(pcl_map.makeShared());
    kd_tree_flann_mutex_.unlock();

    next_astar_ptr_->setObsPcl(pcl_map, 0.5);
}

/**
 * @brief 设置静态地图的KD树
 */
void DWA::setStaticMap(const pcl::PointCloud<pcl::PointXYZ>& static_pcl_map) {
    static_kd_tree_flann_mutex_.lock();
    static_kd_tree_flann_.setInputCloud(static_pcl_map.makeShared());
    static_kd_tree_flann_mutex_.unlock();

    // 设置AStar所使用的地图
    astar_ptr_->setObsPcl(static_pcl_map, 0.25);
}

std::vector<std::vector<DWANodePtr>> DWA::search(const Eigen::Vector3d& start_pos,
                                                 const Eigen::Vector3d& start_vel,
                                                 const Eigen::Vector3d& start_acc,
                                                 const Eigen::Vector3d& end_pos,
                                                 const Eigen::Vector3d& end_vel,
                                                 double                 start_time,
                                                 int                    given_depth) {
    /* 初始化 */
    reset();
    std::vector<std::vector<DWANodePtr>> tree;

    double astar_time {ros::Time::now().toSec()};
    // 使用astar搜索初始可行路径
    astar_ptr_->reset();
    bool                         astar_search_flag {astar_ptr_->searchPath(start_pos, end_pos)};
    std::vector<Eigen::Vector3d> astar_path;
    if (astar_search_flag) {
        astar_path = astar_ptr_->getPath();
    } else {
        astar_path = {end_pos};
    }
    std::cerr << "astar time: " << ros::Time::now().toSec() - astar_time << ", ";
    double test_time {0.0};

    // 创建随机树生成器
    std::random_device rd;
    std::mt19937       gen(rd());

    // 记录开始时间
    start_time_ = start_time;

    // 构建初始节点
    size_t     cur_node_idx = 0;
    DWANodePtr start_ptr    = dwa_node_ptr_vec_[cur_node_idx];
    start_ptr->pos          = start_pos;
    start_ptr->vel          = start_vel;
    start_ptr->acc          = start_acc;
    start_ptr->time         = start_time;
    start_ptr->score        = 0.0;
    start_ptr->cost         = start_ptr->score;
    start_ptr->parent       = nullptr;

    ++cur_node_idx;

    // 根节点
    std::vector<DWANodePtr> tree_root_layer {start_ptr};
    tree.push_back(tree_root_layer);

    /* 探索树，最大搜索深度MAX_TREE_DEPTH */
    for (size_t depth = 1; depth <= given_depth; ++depth) {
        // 上一层搜索树节点集合
        std::vector<DWANodePtr> last_tree_layer = tree[depth - 1];

        // 当前层的代价衰减值
        const double CURRENT_DECLINE_VALUE = std::pow(DECLINE_RATE, depth);

        // 预先计算次方下的时间步长
        double ts1 {time_step_};
        double ts2 {std::pow(time_step_, 2)};
        double ts3 {std::pow(time_step_, 3)};

        // 创建优先级队列，保存代价最小的多个节点
        std::priority_queue<DWANodePtr, std::vector<DWANodePtr>, NodeComparator> pq;
        int                                                                      pq_size {0};

        // 搜索astar路径上，当前时间对应的点（默认最大速度前进）
        double          astar_distance {max_vel_ * time_step_ * depth};
        Eigen::Vector3d ref_pt;
        double          min_astar_dis {1e3};
        for (const Eigen::Vector3d& pt : astar_path) {
            double tmp_dis {(pt - start_pos).norm()};
            if (std::abs(tmp_dis - astar_distance) < min_astar_dis) {
                min_astar_dis = std::abs(tmp_dis - astar_distance);
                ref_pt        = pt;
            }
        }

        // 依次遍历上一层的所有节点
        // FIXME:暂时不考虑z轴
        for (DWANodePtr last_tree_layer_node : last_tree_layer) {
            // 采样加加速度状态空间
            std::vector<double> jerkx = Tools::linspace<double>(-max_jerk_, max_jerk_, SAMPLE_JERK_NUMBER);
            std::vector<double> jerky = Tools::linspace<double>(-max_jerk_, max_jerk_, SAMPLE_JERK_NUMBER);
            // std::vector<double> jerkz = Tools::linspace<double>(-max_jerk_, max_jerk_, sample_jerkz_num_);

            // 根据采样速度进行扩展
            for (double jx : jerkx) {
                for (double jy : jerky) {
                    // for (double jz : jerkz) {

                    // 获取原始节点的状态信息
                    Eigen::Vector3d pos {last_tree_layer_node->pos};
                    Eigen::Vector3d vel {last_tree_layer_node->vel};
                    Eigen::Vector3d acc {last_tree_layer_node->acc};
                    // Eigen::Vector3d jerk(jx, jy, jz);
                    Eigen::Vector3d jerk(jx, jy, 0.0);
                    // 更新状态
                    pos += ts1 * vel + ts2 / 2.0 * acc + ts3 / 6.0 * jerk;
                    vel += ts1 * acc + ts2 / 2.0 * jerk;
                    acc += ts1 * jerk;

                    // 临时使用astar类检查点可行性，其中包含了地图范围检查
                    // std::cerr << "1";
                    if (!astar_ptr_->isPointFeasible(pos)) {
                        // std::cerr << "1\n";
                        continue;
                    }

                    // 限制搜索位置在全局地图范围内部
                    // std::cerr << "2";
                    if (pos.x() < global_map_size_x_min_ || pos.x() > global_map_size_x_max_
                        || pos.y() < global_map_size_y_min_ || pos.y() > global_map_size_y_max_
                        || pos.z() < global_map_size_z_min_ || pos.z() > global_map_size_z_max_) {
                        // std::cerr << "pos: " << pos.transpose() << std::endl;
                        // std::cerr << "2\n";
                        continue;
                    }

                    // 限制与障碍物距离
                    // std::cerr << "3";
                    // ros::Time start_time {ros::Time::now()};
                    // double    static_obstacle_cost = calStaticObstacleCost(pos) * weight_static_obstacle_distance_;
                    // // 与障碍物碰撞或在危险距离内，直接跳过当次
                    // if (static_obstacle_cost < -0.1) {
                    //     // std::cerr << "3\n";
                    //     ros::Time end_time {ros::Time::now()};
                    //     test_time += (end_time - start_time).toSec();
                    //     continue;
                    // }
                    // ros::Time end_time {ros::Time::now()};
                    // test_time += (end_time - start_time).toSec();

                    // 限制速度和加速度
                    // std::cerr << "4";
                    double vel_norm {vel.norm()}, acc_norm {acc.norm()};
                    if (vel_norm > max_vel_ || acc_norm > max_acc_) {
                        // std::cerr << "4\n";
                        continue;
                    }

                    // 计算当前节点对应的时间
                    double time = last_tree_layer_node->time + time_step_;

                    // // 增加一个代价：运行碰撞代价，计算根据当前位置、速度和加速度，运行step时间后的碰撞代价
                    const double next_time {0.5};
                    // // std::cerr << "5";
                    Eigen::Vector3d next_pos = pos + next_time * vel + next_time * next_time / 2.0 * acc
                                               + next_time * next_time * next_time / 6.0 * jerk;
                    // if (!astar_ptr_->isPointFeasible(next_pos)) {
                    //     continue;
                    // }
                    if (!next_astar_ptr_->isPointFeasible(next_pos)) {
                        continue;
                    }
                    // double future_position_cost = calStaticObstacleCost(next_pos) * weight_future_position_;
                    // if (future_position_cost < -0.1) {
                    //     // std::cerr << "5\n";
                    //     continue;
                    // }

                    // 计算距离参考点的距离代价
                    double reference_point_distance_cost {(pos - ref_pt).norm() * weight_reference_point_distance_};

                    // 动态障碍物不确定度信息
                    // std::cerr << "6\n";
                    double obstacle_uncertainty_cost {obs_pred_ptr_->calculateUncertainty(time, pos)
                                                      * weight_dynamic_obstacle_uncertainty_};
                    // 表示碰撞
                    if (obstacle_uncertainty_cost < -0.1) {
                        // std::cerr << "6\n";
                        continue;
                    }

                    // // 启发式距离代价
                    // double heuristic_distance_cost {estimateHeuristic(pos, vel, end_pos, end_vel)
                    //                                 * weight_heuristic_distance_};

                    // 历史代价
                    double history_cost = last_tree_layer_node->cost;

                    // 当前节点代价，包含启发式距离代价、参考距离代价和动态障碍不确定性代价
                    // double current_cost {heuristic_distance_cost + reference_point_distance_cost
                    //                      + obstacle_uncertainty_cost};
                    double current_cost {reference_point_distance_cost + obstacle_uncertainty_cost};
                    // double current_cost {reference_point_distance_cost + static_obstacle_cost
                    //                      + obstacle_uncertainty_cost};
                    // double current_cost {reference_point_distance_cost + obstacle_uncertainty_cost
                    //                      + future_position_cost};

                    // 计算总体代价
                    double cost = history_cost + CURRENT_DECLINE_VALUE * current_cost;

                    // std::cerr << "end\n";

                    // 存入容器中
                    DWANodePtr ptr = dwa_node_ptr_vec_[cur_node_idx];
                    ptr->pos       = pos;
                    ptr->vel       = vel;
                    ptr->time      = time;
                    ptr->score     = current_cost;
                    ptr->cost      = cost;
                    ptr->parent    = last_tree_layer_node;
                    ++cur_node_idx;

                    // 根据层数和已有的节点数据，决定是否存放当前节点
                    if (pq_size < MAX_LAYER_NODE_NUM * (given_depth + 1 - depth)) {
                        // if (pq_size < MAX_LAYER_NODE_NUM) {
                        pq.push(ptr);
                        ++pq_size;
                    } else {
                        if (ptr->cost < pq.top()->cost) {
                            pq.pop();
                            pq.push(ptr);
                        }
                    }
                }
            }
        }

        // std::cerr << ", layer.";

        std::vector<DWANodePtr> full_layer;
        // 如果队列为空，直接返回树
        if (pq.empty()) {
            std::cerr << "test time: " << test_time << ", ";
            return tree;
        } else {
            // 从优先级队列中取出节点，存入容器中
            while (!pq.empty()) {
                full_layer.push_back(pq.top());
                pq.pop();
            }
        }

        /* 自适应降采样 */
        /* 遍历所有情况，将相同索引存放到同一个vector中 */
        double max_x = -inf, max_y = -inf, max_z = -inf;
        double min_x = inf, min_y = inf, min_z = inf;
        // 找到每一轴最大最小值
        for (DWANodePtr& ptr : full_layer) {
            double x = ptr->pos.x(), y = ptr->pos.y(), z = ptr->pos.z();
            if (x < min_x) {
                min_x = x;
            }
            if (x > max_x) {
                max_x = x;
            }
            if (y < min_y) {
                min_y = y;
            }
            if (y > max_y) {
                max_y = y;
            }
            // if (z < min_z) {
            //     min_z = z;
            // }
            // if (z > max_z) {
            //     max_z = z;
            // }
        }
        max_x += 1e-3;
        max_y += 1e-3;
        min_x -= 1e-3;
        min_y -= 1e-3;

        // std::cerr << "2";

        // 获取分别率和最大索引
        double x_res = (max_x - min_x) / static_cast<double>(sample_num_);
        double y_res = (max_y - min_y) / static_cast<double>(sample_num_);
        // double z_res     = (max_z - min_z) / static_cast<double>(sample_num_);
        int                                  max_x_idx = sample_num_, max_y_idx = sample_num_, max_z_idx = sample_num_;
        std::vector<std::vector<DWANodePtr>> full_layer_vec;
        // full_layer_vec.resize(max_x_idx * max_y_idx * max_z_idx);
        full_layer_vec.resize(max_x_idx * max_y_idx);

        // std::cerr << "3";

        // std::cerr << "f" << full_layer.size() << "f";

        // 按照索引存入容器中
        for (DWANodePtr& ptr : full_layer) {
            double x = ptr->pos.x(), y = ptr->pos.y(), z = ptr->pos.z();
            int    x_idx = static_cast<int>((x - min_x) / x_res);
            int    y_idx = static_cast<int>((y - min_y) / y_res);
            // int    z_idx = static_cast<int>((z - min_z) / z_res);
            // int idx = x_idx * max_y_idx * max_z_idx + y_idx * max_z_idx + z_idx;
            int idx = x_idx * max_y_idx + y_idx;
            full_layer_vec[idx].push_back(ptr);
        }

        // std::cerr << "4";

        std::vector<DWANodePtr> current_tree_layer;
        // 进行降采样
        for (std::vector<DWANodePtr>& ptr_vec : full_layer_vec) {
            // 容器有效
            size_t ptr_size = ptr_vec.size();
            if (ptr_size == 1) {
                // 数量为1,直接放入
                current_tree_layer.push_back(ptr_vec.front());
            } else if (ptr_size > 1) {
                // 采样
                std::uniform_int_distribution<size_t> dis(0, ptr_size - 1);
                current_tree_layer.push_back(ptr_vec[dis(gen)]);
            } else {
                // 容器为大小0,不执行操作
            }
        }

        tree.push_back(current_tree_layer);
    }

    std::cerr << "test time: " << test_time << ", ";

    return tree;
}

/**
 * @brief 根据输入的搜索树，返回最优成本代价的节点路径
 */
std::vector<DWANodePtr> DWA::getBestNodePath(const std::vector<std::vector<DWANodePtr>>& tree) {
    // 获取最后一层搜索树节点集合
    std::vector<DWANodePtr> last_tree_layer = tree.back();
    // 根据代价进行排序
    std::sort(last_tree_layer.begin(), last_tree_layer.end(), NodeComparator());

    // 选择第一个，即代价最小的节点进行回溯，直到根节点
    std::vector<DWANodePtr> node_path;
    DWANodePtr              current_ptr = last_tree_layer.front();
    while (current_ptr->parent != nullptr) {
        node_path.push_back(current_ptr);
        current_ptr = current_ptr->parent;
    }
    node_path.push_back(current_ptr);
    // 反转vector，变为从根节点到末尾节点
    std::reverse(node_path.begin(), node_path.end());

    return node_path;
}

/**
 * @brief 根据输入的搜索树，返回最优成本代价的节点路径
 */
std::vector<Eigen::Vector3d> DWA::getBestPath(const std::vector<std::vector<DWANodePtr>>& tree) {
    // 获取最后一层搜索树节点集合
    std::vector<DWANodePtr> last_tree_layer = tree.back();

    // 根据代价进行排序
    std::sort(last_tree_layer.begin(), last_tree_layer.end(), NodeComparator());

    // 选择第一个，即代价最小的节点进行回溯，直到根节点
    std::vector<Eigen::Vector3d> path;
    DWANodePtr                   current_ptr = last_tree_layer.front();
    while (current_ptr->parent != nullptr) {
        path.push_back(current_ptr->pos);
        current_ptr = current_ptr->parent;
    }

    path.push_back(current_ptr->pos);
    // 反转vector，变为从根节点到末尾节点
    std::reverse(path.begin(), path.end());

    return path;
}

/**
 * @brief 根据输入的搜索树，返回最优成本代价的组合路径
 */
std::vector<std::tuple<Eigen::Vector3d, double>> DWA::getBestTimePath(
    const std::vector<std::vector<DWANodePtr>>& tree) {
    // 获取最后一层搜索树节点集合
    std::vector<DWANodePtr> last_tree_layer = tree.back();
    // 根据代价进行排序
    std::sort(last_tree_layer.begin(), last_tree_layer.end(), NodeComparator());

    // 选择第一个，即代价最小的节点进行回溯，直到根节点
    std::vector<std::tuple<Eigen::Vector3d, double>> time_path;
    DWANodePtr                                       current_ptr = last_tree_layer.front();
    while (current_ptr->parent != nullptr) {
        std::tuple tuple_state {current_ptr->pos, current_ptr->time};
        time_path.push_back(tuple_state);
        current_ptr = current_ptr->parent;
    }
    time_path.emplace_back(current_ptr->pos, current_ptr->time);
    // 反转vector，变为从根节点到末尾节点
    std::reverse(time_path.begin(), time_path.end());

    return time_path;
}

/**
 * @brief 根据输入的搜索树，返回最优成本代价的组合路径
 */
std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> DWA::getBestTuplePath(
    const std::vector<std::vector<DWANodePtr>>& tree) {
    // 获取最后一层搜索树节点集合
    std::vector<DWANodePtr> last_tree_layer = tree.back();
    // 根据代价进行排序
    std::sort(last_tree_layer.begin(), last_tree_layer.end(), NodeComparator());

    // 选择第一个，即代价最小的节点进行回溯，直到根节点
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>> tuple_path;
    DWANodePtr                                                                 current_ptr = last_tree_layer.front();
    while (current_ptr->parent != nullptr) {
        std::tuple tuple_state {current_ptr->pos, current_ptr->vel, current_ptr->acc};
        tuple_path.push_back(tuple_state);
        current_ptr = current_ptr->parent;
    }
    tuple_path.emplace_back(current_ptr->pos, current_ptr->vel, current_ptr->acc);
    // 反转vector，变为从根节点到末尾节点
    std::reverse(tuple_path.begin(), tuple_path.end());

    return tuple_path;
}

double DWA::estimateHeuristic(const Eigen::Vector3d& start_pos,
                              const Eigen::Vector3d& start_vel,
                              const Eigen::Vector3d& end_pos,
                              const Eigen::Vector3d& end_vel) {
    // MODIFIED: 后续研究

    const Eigen::Vector3d dp = end_pos - start_pos;  // 位置差
    const Eigen::Vector3d v0 = start_vel;            // 起始速度
    const Eigen::Vector3d v1 = end_vel;              // 终止速度

    double c1 = -36.0 * dp.dot(dp);
    double c2 = 24.0 * (v0 + v1).dot(dp);
    double c3 = -4.0 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0.0;
    double c5 = w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);  // 求根

    double v_max = max_vel_ * 0.5;
    // Uniform deceleration
    double t_bar = (start_pos - end_pos).lpNorm<Eigen::Infinity>() / v_max;
    ts.push_back(t_bar);

    // Set cost +inf.
    double cost = inf;
    double t_d  = t_bar;

    // 找到最优代价（距离）
    for (double t : ts) {
        // 时间必须超过0
        if (t < t_bar) {
            continue;
        }

        // 使用根求解代价
        double c = -c1 / (3.0 * t * t * t) - c2 / (2.0 * t * t) - c3 / t + w_time_ * t;

        // 找到最小代价，并更新
        if (c < cost) {
            cost = c;
            t_d  = t;
        }
    }

    return cost;
}

std::vector<double> DWA::cubic(double a, double b, double c, double d) {
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3.0 * a1 - a2 * a2) / 9.0;
    double R = (9.0 * a1 * a2 - 27.0 * a0 - 2.0 * a2 * a2 * a2) / 54.0;
    double D = Q * Q * Q + R * R;
    if (D > 0) {
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        dts.push_back(-a2 / 3.0 + (S + T));
        return dts;
    } else if (D == 0) {
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3.0 + S + S);
        dts.push_back(-a2 / 3.0 - S);
        return dts;
    } else {
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2.0 * sqrt(-Q) * cos(theta / 3) - a2 / 3.0);
        dts.push_back(2.0 * sqrt(-Q) * cos((theta + 2.0 * M_PI) / 3.0) - a2 / 3.0);
        dts.push_back(2.0 * sqrt(-Q) * cos((theta + 4.0 * M_PI) / 3.0) - a2 / 3.0);
        return dts;
    }
}

std::vector<double> DWA::quartic(double a, double b, double c, double d, double e) {
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1.0, -a2, a1 * a3 - 4.0 * a0, 4.0 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double              y1 = ys.front();
    double              r  = a3 * a3 / 4.0 - a2 + y1;
    if (r < 0.0) {
        return dts;
    }

    double R = sqrt(r);
    double D, E;
    if (R != 0.0) {
        D = sqrt(0.75 * a3 * a3 - R * R - 2.0 * a2 + 0.25 * (4.0 * a3 * a2 - 8.0 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2.0 * a2 - 0.25 * (4.0 * a3 * a2 - 8.0 * a1 - a3 * a3 * a3) / R);
    } else {
        D = sqrt(0.75 * a3 * a3 - 2.0 * a2 + 2.0 * sqrt(y1 * y1 - 4.0 * a0));
        E = sqrt(0.75 * a3 * a3 - 2.0 * a2 - 2.0 * sqrt(y1 * y1 - 4.0 * a0));
    }

    if (!std::isnan(D)) {
        dts.push_back(-a3 / 4.0 + R / 2.0 + D / 2.0);
        dts.push_back(-a3 / 4.0 + R / 2.0 - D / 2.0);
    }
    if (!std::isnan(E)) {
        dts.push_back(-a3 / 4.0 - R / 2.0 + E / 2.0);
        dts.push_back(-a3 / 4.0 - R / 2.0 - E / 2.0);
    }

    return dts;
}

/**
 * @brief 状态空间采样
 */
std::vector<DWANodePtr> DWA::stateExpand(const DWANodePtr& node, int sample_num) {
    // 速度空间采样
    double min_vx = std::max(-max_vel_, node->vel.x() - time_step_ * max_acc_);
    double max_vx = std::min(max_vel_, node->vel.x() + time_step_ * max_acc_);
    double min_vy = std::max(-max_vel_, node->vel.y() - time_step_ * max_acc_);
    double max_vy = std::min(max_vel_, node->vel.y() + time_step_ * max_acc_);
    double min_vz = std::max(-max_vel_, node->vel.z() - time_step_ * max_acc_);
    double max_vz = std::min(max_vel_, node->vel.z() + time_step_ * max_acc_);

    std::vector<double> velx = Tools::linspace<double>(min_vx, max_vx, sample_num);
    std::vector<double> vely = Tools::linspace<double>(min_vy, max_vy, sample_num);
    std::vector<double> velz = Tools::linspace<double>(min_vz, max_vz, sample_num);

    // 预分配vector
    std::vector<DWANodePtr> expanded_states;
    expanded_states.reserve(velx.size() * vely.size() * velz.size());

    for (double vx : velx) {
        for (double vy : vely) {
            for (double vz : velz) {
                // 生成新节点
                DWANodePtr new_node = std::make_shared<DWANode>();
                new_node->vel       = Eigen::Vector3d(vx, vy, vz);
                new_node->pos       = node->pos + time_step_ * new_node->vel;
                new_node->time      = node->time + time_step_;
                // new_node->score   = node->g_score + time_step_ * new_node->vel.norm();
                new_node->parent = node;
                // 放入新节点
                expanded_states.push_back(new_node);
            }
        }
    }

    return expanded_states;
}

/**
 * @brief 将位置信息转化为int型的位置索引
 *
 * @return 位置索引
 * @retval int
 */
int DWA::posToIndex(const Eigen::Vector3d& pos) {
    // 计算偏离值
    Eigen::Vector3d delta_pos = pos - center_;

    /* 计算索引 */
    // 根据delta_pos计算索引
    Eigen::Vector3i delta_pos_idx = ((delta_pos - low_bound_) / POSE_INDEX_RESOLUTION).cast<int>();

    // 限制索引在0到地图最大索引范围内部
    int idx_x = std::clamp(delta_pos_idx.x(), 0, map_size_.x() - 1);
    int idx_y = std::clamp(delta_pos_idx.y(), 0, map_size_.y() - 1);
    int idx_z = std::clamp(delta_pos_idx.z(), 0, map_size_.z() - 1);

    return idx_x * pos_max_idx_.y() * pos_max_idx_.z() + idx_y * pos_max_idx_.z() + idx_z;
}

/**
 * @brief 时间索引
 */
int DWA::timeToIndex(double time) {
    return static_cast<int>((time - start_time_) / TIME_INDEX_RESOLUTION);
}

/**
 * @brief 计算节点代价，返回代价值
 *
 * @param cur_pos 当前位置
 * @param cur_vel 当前速度
 * @param end_pos 终点位置
 * @param end_vel 终点速度
 *
 * @return 计算得到的节点代价值
 */
double DWA::calCost(const Eigen::Vector3d& cur_pos,
                    const Eigen::Vector3d& cur_vel,
                    const Eigen::Vector3d& end_pos,
                    const Eigen::Vector3d& end_vel) {
    // 距离普通障碍物的风险代价
    double static_obstacle_distance_cost = calStaticObstacleCost(cur_pos);
    if (static_obstacle_distance_cost < -1e-3) {
        // 代价为负数表示发生碰撞，同样返回负数代价
        return -1.0;
    }

    // TODO:距离动态障碍物的风险代价

    // 距离终点的启发式距离代价
    double heuristic_distance_cost = lambda_heu_ * estimateHeuristic(cur_pos, cur_vel, end_pos, end_vel);

    // TODO:增加权重
    // return static_obstacle_distance_cost + heuristic_distance_cost;
    return heuristic_distance_cost;
}

/**
 * @brief 计算当前节点距离静态普通障碍物的代价，并返回代价值
 */
double DWA::calStaticObstacleCost(const Eigen::Vector3d& pos) {
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = pos.x(), pcl_pt.y = pos.y(), pcl_pt.z = pos.z();

    // 使用KD树搜索最近的1个障碍点，获得距离
    pcl::Indices       kd_tree_point_indices;
    std::vector<float> kd_tree_point_distance;

    // std::cerr << "yes1.";

    // 最近邻搜索当前点距离点云最近的距离
    static_kd_tree_flann_mutex_.lock();
    int found_number = static_kd_tree_flann_.nearestKSearch(pcl_pt, 1, kd_tree_point_indices, kd_tree_point_distance);
    static_kd_tree_flann_mutex_.unlock();

    // std::cerr << "yes2.";

    // 如果没有找到最近邻点，默认此时距离普通障碍物有一定距离，代价为0
    if (found_number == 0) {
        return 0.0;
    }

    // 提取距离值并转为double类型
    double obstacle_distance = static_cast<double>(kd_tree_point_distance.front());
    if (obstacle_distance < MIN_SAFE_DISTANCE) {
        // 如果距离障碍物的距离小于安全距离，返回碰撞代价
        return HIT_COST;
    } else if (obstacle_distance > MAX_COST_DISTANCE) {
        // 如果距离障碍物的距离大于设定的最大计算代价距离，认为此时没有代价
        return 0.0;
    } else {
        // 在else外部设定最终逻辑，便于保证返回值
    }

    return std::pow(MAX_COST_DISTANCE - obstacle_distance, 2);
}

/**
 * @brief 检查给定点是否与障碍物碰撞
 *
 * @return true表示占据，不可行；false表示该位置可行
 */
bool DWA::isOccupied(const Eigen::Vector3d& pos) {
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = pos.x(), pcl_pt.y = pos.y(), pcl_pt.z = pos.z();

    // 使用KD树搜索最近的1个障碍点，获得距离
    pcl::Indices       kd_tree_point_indices;
    std::vector<float> kd_tree_point_distance;

    // 最近邻搜索当前点距离点云最近的距离
    kd_tree_flann_mutex_.lock();
    int found_number = kd_tree_flann_.nearestKSearch(pcl_pt, 1, kd_tree_point_indices, kd_tree_point_distance);
    kd_tree_flann_mutex_.unlock();

    // 如果没有找到最近邻点，默认此时距离普通障碍物有一定距离，未碰撞
    if (found_number == 0) {
        return false;
    }

    // 提取距离值并转为double类型
    double obstacle_distance = static_cast<double>(kd_tree_point_distance.front());
    if (obstacle_distance < MIN_SAFE_DISTANCE) {
        // 如果距离障碍物的距离小于安全距离，认为碰撞
        return true;
    }

    // 找到最近临点，且距离大于安全距离，认为没有碰撞
    return false;
}

/**
 * @brief 设置障碍预测类指针
 */
void DWA::setObstaclePrediction(ObstaclesPredictionPtr ptr) {
    obs_pred_ptr_ = ptr;
}
