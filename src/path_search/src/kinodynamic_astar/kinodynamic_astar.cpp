/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 *
 * 1. Delete reach_horizon
 */

// TODO: 修改Kinodynamic Astar为加加速度变化，加速度保持连续

#include "kinodynamic_astar/kinodynamic_astar.h"

#include <cmath>

#include "node/node.h"

/**
 * @brief 初始化KinoAstar的类
 */
KinodynamicAstar::KinodynamicAstar() {
    // 搜索用参数
    use_node_num_ = 0, iter_num_ = 0;
    start_time_ = 0.0;

    // 限制参数
    max_vel_ = 10.0, max_acc_ = 20.0;

    // 检查可行性参数
    time_resolution_ = 0.1;
    check_num_       = 10;  // KinoAstar采样点数量，用于检查碰撞

    // 权重
    lambda_heu_ = 5.0, w_time_ = 10.0;
    max_tau_     = 0.6;
    tie_breaker_ = 1.0 + 1.0 / 1e4;

    // 防止指针悬空
    terminate_ptr_ = nullptr;
    obs_pred_ptr_  = nullptr;
}

/**
 * @param resolution 分辨率
 * @param gl_low 全局地图的最小坐标
 * @param gl_upp 全局地图的最大坐标
 */
void KinodynamicAstar::initMap(const double resolution, const Eigen::Vector3d& gl_low, const Eigen::Vector3d& gl_upp) {
    center_     = Eigen::Vector3d::Zero();
    resolution_ = resolution;
    gl_low_     = gl_low;
    gl_upp_     = gl_upp;
    gl_size_    = ((gl_upp - gl_low) / resolution_).cast<int>();

    // 预先定义内部栅格地图的大小，FREE表示没有占用，OCCUPIED表示占用
    grid_map_.resize(gl_size_.prod(), FREE);
}

/**
 * @brief 设置障碍预测类ObstaclesPrediction
 */
void KinodynamicAstar::setObstaclesPrediction(ObstaclesPredictionPtr ptr) {
    obs_pred_ptr_ = ptr;
}

/**
 * @brief 重置地图节点，相关数据参数
 */
void KinodynamicAstar::reset(void) {
    // 清空优先级队列
    std::priority_queue<KinoAstarNodePtr, std::vector<KinoAstarNodePtr>, NodeComparator> empty_priority_queue;
    open_set_.swap(empty_priority_queue);
    // 清空哈希表
    node_hash_table_.clear();
}

/**
 * @brief 设置地图中心点
 */
void KinodynamicAstar::setCenter(const Eigen::Vector3d& center) {
    center_ = center;
}

/**
 * @brief 判断给定点是否在地图中
 */
bool KinodynamicAstar::isInMap(const Eigen::Vector3d& pt) {
    // 计算与中心点的偏差
    Eigen::Vector3d error = pt - center_;
    // 判断是否在给定的最大值与最小值范围内
    if (error.x() > gl_low_.x() && error.x() < gl_upp_.x() && error.y() > gl_low_.y() && error.y() < gl_upp_.y()
        && error.z() > gl_low_.z() && error.z() < gl_upp_.z()) {
        return true;
    }
    return false;
}

/**
 * @brief 重置地图为可用
 */
void KinodynamicAstar::resetGridMap() {
    std::fill(grid_map_.begin(), grid_map_.end(), FREE);
}

/**
 * @brief 为地图设置单个障碍
 */
void KinodynamicAstar::setObs(double coord_x, double coord_y, double coord_z) {
    // 根据中心点计算局部地图中的位置
    coord_x -= center_.x();
    coord_y -= center_.y();
    coord_z -= center_.z();

    // 超出地图范围，不设置障碍
    if (coord_x < gl_low_.x() || coord_y < gl_low_.y() || coord_z < gl_low_.z() || coord_x > gl_upp_.x()
        || coord_y > gl_upp_.y() || coord_z > gl_upp_.z()) {
        return;
    }

    // 获取障碍坐标在地图中的索引坐标
    const int x = static_cast<int>((coord_x - gl_low_.x()) / resolution_);
    const int y = static_cast<int>((coord_y - gl_low_.y()) / resolution_);
    const int z = static_cast<int>((coord_z - gl_low_.z()) / resolution_);
    grid_map_[x * gl_size_.y() * gl_size_.z() + y * gl_size_.z() + z] = OCCUPIED;
}

/**
 * @brief 根据输入的vector形式的障碍点，设置障碍
 */
void KinodynamicAstar::setObsVector(std::vector<Eigen::Vector3d>& cloud, double radius) {
    // 遍历给定的障碍点
    for (const Eigen::Vector3d& pt : cloud) {
        // 增加膨胀，设置障碍
        for (double x = -radius; x <= radius; x += resolution_) {
            for (double y = -radius; y <= radius; y += resolution_) {
                for (double z = -radius; z <= radius; z += resolution_) {
                    setObs(pt.x() + x, pt.y() + y, pt.z() + z);
                }
            }
        }
    }
}

/**
 * @brief 将给定的坐标转换为索引坐标
 */
Eigen::Vector3i KinodynamicAstar::coordToGridIndex(const Eigen::Vector3d& pos) {
    // 计算偏离
    Eigen::Vector3d error_pos = pos - center_;
    Eigen::Vector3i index;
    // 确保在地图范围内，如果超出范围，则设置为起点与给定点在边界上的点
    index.x() = std::min(std::max(static_cast<int>((error_pos.x() - gl_low_.x()) / resolution_), 0), gl_size_.x() - 1);
    index.y() = std::min(std::max(static_cast<int>((error_pos.y() - gl_low_.y()) / resolution_), 0), gl_size_.y() - 1);
    index.z() = std::min(std::max(static_cast<int>((error_pos.z() - gl_low_.z()) / resolution_), 0), gl_size_.z() - 1);
    return index;
}

/**
 * @brief 检查地图占用情况，对于地图外的点，默认没有占用
 *
 * @param pt_idx 给定需要检查的点索引
 *
 * @return 处于占用返回true，处于空闲返回false
 */
bool KinodynamicAstar::isOccupied(const Eigen::Vector3i& idx) {
    // // TEST
    // if (idx.x() >= 0 && idx.x() < gl_size_.x() && idx.y() >= 0 && idx.y() < gl_size_.y() && idx.z() >= 0
    //     && idx.z() < gl_size_.z()) {
    //     std::cerr << "In map!\n";
    // }
    return idx.x() >= 0 && idx.x() < gl_size_.x() && idx.y() >= 0 && idx.y() < gl_size_.y() && idx.z() >= 0
           && idx.z() < gl_size_.z()
           && grid_map_[idx.x() * gl_size_.y() * gl_size_.z() + idx.y() * gl_size_.z() + idx.z()] == OCCUPIED;
}

/**
 * @brief 检查给定的点是否可行。要求在地图中同时未被占据，认为该点可行。
 *
 * @return 在地图中，且未占用，返回true，否则返回false
 */
bool KinodynamicAstar::isPointFeasible(const Eigen::Vector3d& pt, const double time) {
    // 检查给定点是否在地图中
    if (!isInMap(pt)) {
        return false;
    }

    // 检查给定点占据情况(静态地图)
    if (isOccupied(coordToGridIndex(pt))) {
        return false;
    }

    // 检查动态障碍物占据情况
    if (obs_pred_ptr_->isMultiOccupied(time, pt)) {
        return false;
    }

    // 在地图中，且未被占用，返回true
    return true;
}

/**
 * @brief 检查给定的点是否可行。要求在地图中同时未被占据，认为该点可行。
 *
 * @return 在地图中，且未占用，返回true，否则返回false
 */
bool KinodynamicAstar::isPointFeasible(const std::pair<double, Eigen::Vector3d>& time_pos) {
    return isPointFeasible(time_pos.second, time_pos.first);
}

/**
 * @brief 检查从节点出发，根据输入和指定的持续时间后中，采样路径的可行性
 *
 * @param cur_pos 当前位置
 * @param cur_vel 当前速度
 * @param cur_time 当前的时间
 * @param input 当前的输出
 * @param duration 当前节点的持续时间（到达下一个节点）
 */
bool KinodynamicAstar::isNodeSampledPathFeasible(const Eigen::Vector3d& cur_pos,
                                                 const Eigen::Vector3d& cur_vel,
                                                 const double           cur_time,
                                                 const Eigen::Vector3d& input,
                                                 const double           duration) {
    // 设置采样点
    for (int i = 1; i <= check_num_; ++i) {
        const double dt                 = duration * static_cast<double>(i) / static_cast<double>(check_num_);
        const double sampled_time       = cur_time + dt;
        auto [sampled_pos, sampled_vel] = stateTransit(cur_pos, cur_vel, input, dt);

        // 检查静态地图
        if (isOccupied(posToIndex(sampled_pos))) {
            // std::cerr << "posToIndex: " << posToIndex(sampled_pos).transpose() << std::endl;
            return false;
        }

        // 检查动态地图
        if (obs_pred_ptr_->isMultiOccupied(sampled_time, sampled_pos)) {
            return false;
        }
    }

    // 均没有碰撞，返回false
    return true;
}

// /**
//  * @brief 检查pt1到pt2组成的线，是否可行
//  */
// bool KinodynamicAstar::isLineFeasible(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2) {
//     const Eigen::Vector3d err_vec   = pt2 - pt1;
//     const int             point_num = static_cast<int>(err_vec.norm() / resolution_);

//     // 根据设定的采样点个数，检查从pt1到pt2上的采样点是否可行
//     for (int i = 1; i <= point_num; ++i) {
//         const Eigen::Vector3d coord = pt1 + err_vec * static_cast<double>(i) / static_cast<double>(point_num + 1);
//         if (!isPointFeasible(coord)) {
//             return false;
//         }
//     }
//     return true;
// }

/**
 * @brief 检查由多个点组成的轨迹段，是否可行
 */
bool KinodynamicAstar::isPathFeasible(const std::vector<std::pair<double, Eigen::Vector3d>>& time_path) {
    // 轨迹段为空，默认可行
    if (time_path.size() == 0) {
        return true;
    }

    for (const std::pair<double, Eigen::Vector3d>& time_pos : time_path) {
        // 检查给定点在对应时间处是否可行
        if (!isPointFeasible(time_pos)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 使用Kino Astar搜索路径
 *
 * @return 返回搜索状态
 */
KinodynamicAstar::KinoState KinodynamicAstar::search(const Eigen::Vector3d& start_pos,
                                                     const Eigen::Vector3d& start_vel,
                                                     const Eigen::Vector3d& start_acc,
                                                     const Eigen::Vector3d& end_pos,
                                                     const Eigen::Vector3d& end_vel,
                                                     double                 start_time) {
    // 初始化相关参数
    reset();  // 清空open_set_和node_hash_table_
    is_shot_succ_ = false;
    use_node_num_ = 0;
    iter_num_     = 0;

    // 记录开始时间
    start_time_ = start_time;
    // 初始化到达终点时间
    double time_to_goal;

    // 初始化终点状态
    end_pos_                = end_pos;
    Eigen::Vector3i end_idx = posToIndex(end_pos);

    // 初始化起始节点
    KinoAstarNodePtr start_ptr = std::make_shared<KinoAstarNode>();
    start_ptr->pos             = start_pos;
    start_ptr->vel             = start_vel;
    start_ptr->time            = start_time;
    start_ptr->g_score         = 0.0;
    start_ptr->f_score         = lambda_heu_ * estimateHeuristic(start_pos, start_vel, end_pos, end_vel, time_to_goal);
    start_ptr->parent          = nullptr;
    start_ptr->node_state      = OPENED;
    // 起始节点索引
    const Eigen::Vector3i start_idx      = posToIndex(start_pos);
    const int             start_time_idx = timeToIndex(start_time);
    // start_ptr->state           = start_state;
    // start_ptr->index      = posToIndex(start_pos);
    // start_ptr->time       = start_time;
    // start_ptr->time_index = timeToIndex(start_time);
    // start_ptr->g_score    = 0.0;
    // start_ptr->f_score    = lambda_heu_ * estimateHeuristic(start_state, end_state, time_to_goal);
    // start_ptr->parent     = nullptr;
    // start_ptr->node_state = OPENED;

    // 将起始节点插入开集中
    open_set_.push(start_ptr);
    // 使用哈希表存储节点信息
    node_hash_table_.insert(start_idx, start_time_idx, start_ptr);
    // node_hash_table_.insert(start_ptr->index, start_ptr->time_index, start_ptr);  // 使用哈希表存储所有节点信息
    // 使用节点数增加1
    ++use_node_num_;

    // 判断终止的容忍度（比较索引值）
    // const int tolerance = static_cast<int>(1.0 / resolution_);
    const int tolerance = 10;

    // 第一次搜索，不改变加加速度值，以保证连贯性
    bool init_search = true;

    // 开集非空则持续运行
    while (!open_set_.empty()) {
        // ROS_INFO("Open set size: %zu.", open_set_.size());
        // 获得f值最小的节点
        KinoAstarNodePtr cur_ptr = open_set_.top();
        // Eigen::Vector3d  tmp_pos = current_ptr->state.head(3);
        // ROS_INFO("Current node position: %f, %f, %f", tmp_pos.x(), tmp_pos.y(), tmp_pos.z());

        // 将当前节点从开集中移除，并设置为已探索
        open_set_.pop();
        cur_ptr->node_state = CLOSED;
        // 增加迭代次数
        ++iter_num_;

        // 终止条件：每轴的差值均小于容忍度
        Eigen::Vector3i diff_stop = posToIndex(cur_ptr->pos) - end_idx;
        bool            near_end =
            abs(diff_stop.x()) <= tolerance && abs(diff_stop.y()) <= tolerance && abs(diff_stop.z()) <= tolerance;

        // 如果超出搜索次数，以当前情况为终止点，给出路径
        if (iter_num_ > MAX_ITER_NUM) {
            terminate_ptr_ = cur_ptr;
            computeShotTraj(terminate_ptr_->pos, terminate_ptr_->vel, end_pos, end_vel, time_to_goal);
            if (terminate_ptr_->parent != nullptr) {
                return KinoState::REACH_END;
            } else {
                return KinoState::NO_PATH;
            }
        }

        // 如果接近终点
        if (near_end) {
            // 确定终止节点指针
            terminate_ptr_ = cur_ptr;
            // return KinoState::REACH_END;

            // Check whether shot trajectory exist or not.
            computeShotTraj(terminate_ptr_->pos, terminate_ptr_->vel, end_pos, end_vel, time_to_goal);

            // 如果搜索到末端可行轨迹，设置is_shot_succ_为true，
            // 认为只要搜索到路径，均视为到达终点
            if (terminate_ptr_->parent != nullptr) {
                return KinoState::REACH_END;
            } else {
                return KinoState::NO_PATH;
            }

            // if (is_shot_succ_) {
            //     return KinoState::REACH_END;
            // } else if (terminate_ptr_->parent != nullptr) {  // 仅仅只是靠近终点
            //     // std::cout << "[Kino Astar] Near end!" << std::endl;
            //     return KinoState::NEAR_END;
            // } else {  // 没有搜索到可行路径
            //     std::cout << "[Kino Astar] No path!" << std::endl;
            //     return KinoState::NO_PATH;
            // }
        }

        // 确定当前状态
        Eigen::Vector3d cur_pos      = cur_ptr->pos;
        Eigen::Vector3d cur_vel      = cur_ptr->vel;
        double          cur_time     = cur_ptr->time;
        Eigen::Vector3i cur_idx      = posToIndex(cur_pos);
        int             cur_time_idx = timeToIndex(cur_time);
        double          cur_g_score  = cur_ptr->g_score;
        double          cur_f_score  = cur_ptr->f_score;
        // Eigen::Matrix<double, 6, 1> cur_state    = current_ptr->state;

        /* 控制输入与持续时间 */
        double                       res      = 1.0 / 5.0;                              // 分辨率
        double                       time_res = 1.0 / 5.0, time_init_res = 1.0 / 10.0;  // 时间分辨率
        std::vector<Eigen::Vector3d> inputs;
        std::vector<double>          durations;
        // TODO:考虑加速度的给定方式
        if (init_search) {
            // 初始搜索下，不更改加速度，仅更新时间
            inputs.push_back(start_acc);
            // 初始情况下，采样时间更多，考虑更多情况
            for (double tau = time_init_res * max_tau_; tau <= max_tau_ + 1e-3; tau += time_init_res * max_tau_) {
                durations.push_back(tau);
            }
            init_search = false;
        } else {  // 后续考虑加加速度的变化情况
            // 采样加速度变化情况
            for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res) {
                for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res) {
                    for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res) {
                        inputs.emplace_back(ax, ay, az);
                    }
                }
            }
            // 计算持续时间
            for (double tau = time_res * max_tau_; tau <= max_tau_ + 1e-3; tau += time_res * max_tau_) {
                durations.push_back(tau);
            }
            // TODO:后续可以考虑更新为偏移量计算
            // for (double ax = -0.1; ax <= 0.1; ax += 0.05) {
            //     for (double ay = -0.1; ay <= 0.1; ay += 0.05) {
            //         for (double az = -0.1; az <= 0.1; az += 0.05) {
            //             inputs.push_back(Eigen::Vector3d(ax, ay, az));
            //         }
            //     }
            // }
        }

        // ROS_INFO("Inputs size: %zu, durations size: %zu", inputs.size(), durations.size());

        /* 获取邻居节点 */
        // 遍历每个控制输入
        // int tmp_cnt_vel = 0, tmp_cnt_diff = 0;
        for (const Eigen::Vector3d& input : inputs) {
            // 遍历每个持续时间
            for (const double duration : durations) {
                // 计算在控制输入input和持续时间duration下的下一个状态
                auto [expanded_pos, expanded_vel] = stateTransit(cur_pos, cur_vel, input, duration);

                // Eigen::Matrix<double, 6, 1> expanded_state = stateTransit(cur_state, input, duration);

                // std::cout << "pos: " << expanded_state.head(3).transpose()
                //           << ", vel: " << expanded_state.tail(3).transpose() << std::endl;

                /* 检查可行性 */
                // 只要存在某一轴超过最大速度限制的情况，则跳过当前情况
                if (std::abs(expanded_vel.x()) > max_vel_ || std::abs(expanded_vel.y()) > max_vel_
                    || std::abs(expanded_vel.z()) > max_vel_) {
                    // ++tmp_cnt_vel;
                    // std::cout << "over vel: " << expanded_vel.transpose() << std::endl;
                    continue;
                }

                // 检查扩展节点和当前节点是否位于同一个栅格内部，不是则跳过
                Eigen::Vector3i expanded_idx      = posToIndex(expanded_pos);
                double          expanded_time     = cur_time + duration;
                int             expanded_time_idx = timeToIndex(expanded_time);
                // 检查位置和时间的索引即可
                Eigen::Vector3i diff_idx      = expanded_idx - cur_idx;
                int             diff_time_idx = expanded_time_idx - cur_time_idx;
                if (diff_idx.x() == 0 && diff_idx.y() == 0 && diff_idx.z() == 0 && diff_time_idx == 0) {
                    // 位于同一栅格内，跳过
                    // ++tmp_cnt_diff;
                    continue;
                }

                // TODO:后续再加上对节点可靠性的考虑（不确定度）

                // if (init_search || true) {
                //     std::cout << expanded_vel.transpose() << std::endl;
                // }

                // 检查节点可行性（碰撞）
                if (!isNodeSampledPathFeasible(cur_pos, cur_vel, cur_time, input, duration)) {
                    // std::cerr << "Current state: " << cur_state.transpose() << std::endl;
                    // std::cerr << "Current time: " << cur_time << std::endl;
                    // std::cerr << "Current input: " << input.transpose() << std::endl;
                    // std::cerr << "Current duration: " << duration << std::endl;
                    continue;
                }

                // 计算扩展节点的g值和f值
                double expanded_g_score = (input.squaredNorm() + w_time_) * duration + cur_g_score;
                double expanded_f_score =
                    expanded_g_score
                    + lambda_heu_ * estimateHeuristic(expanded_pos, expanded_vel, end_pos, end_vel, time_to_goal);

                // 搜索节点是否已经存在
                KinoAstarNodePtr expanded_ptr = node_hash_table_.find(expanded_idx, expanded_time_idx);
                if (expanded_ptr != nullptr) {
                    // 节点存在
                    // 如果节点状态为开集，对比新旧节点的f值
                    if (expanded_ptr->node_state == OPENED) {
                        // 更新最小f值的节点
                        if (expanded_f_score < expanded_ptr->f_score) {
                            expanded_ptr->pos      = expanded_pos;
                            expanded_ptr->vel      = expanded_vel;
                            expanded_ptr->time     = expanded_time;
                            expanded_ptr->input    = input;
                            expanded_ptr->duration = duration;
                            expanded_ptr->g_score  = expanded_g_score;
                            expanded_ptr->f_score  = expanded_f_score;
                            expanded_ptr->parent   = cur_ptr;
                            // expanded_ptr->state   = expanded_state;
                        }
                    } else {  // 节点状态为闭集，跳过
                        continue;
                    }
                } else {  // 节点不存在，为nullptr
                    // 创建节点，并设置状态为开
                    expanded_ptr             = std::make_shared<KinoAstarNode>();
                    expanded_ptr->pos        = expanded_pos;
                    expanded_ptr->vel        = expanded_vel;
                    expanded_ptr->time       = expanded_time;
                    expanded_ptr->duration   = duration;
                    expanded_ptr->input      = input;
                    expanded_ptr->g_score    = expanded_g_score;
                    expanded_ptr->f_score    = expanded_f_score;
                    expanded_ptr->parent     = cur_ptr;
                    expanded_ptr->node_state = OPENED;
                    // expanded_ptr->index      = posToIndex(expanded_state.head(3));
                    // expanded_ptr->time_index = expanded_time_idx;
                    // expanded_ptr->duration   = duration;
                    // expanded_ptr->input      = input;
                    // expanded_ptr->state      = expanded_state;
                    // 插入开集中
                    open_set_.push(expanded_ptr);
                    // 插入节点哈希表中
                    node_hash_table_.insert(expanded_idx, expanded_time_idx, expanded_ptr);
                    // 增加使用节点个数
                    ++use_node_num_;
                    // std::cout << "YES.\n";
                }
            }
        }
        // init_search = false;
        // std::cout << "tmp cnt vel: " << tmp_cnt_vel << ", tmp cnt diff: " << tmp_cnt_diff << std::endl;
    }

    std::cout << "\033[31mOpen set empty, no path!\033[0m" << std::endl;
    std::cout << "\033[31muse node num: " << use_node_num_ << "\033[0m" << std::endl;
    std::cout << "\033[31miter num: " << iter_num_ << "\033[0m" << std::endl;
    return KinoState::NO_PATH;
}

/**
 * @brief 坐标转换为索引坐标
 */
Eigen::Vector3i KinodynamicAstar::posToIndex(const Eigen::Vector3d& pos) {
    // 计算偏离
    Eigen::Vector3d delta_pos = pos - center_;
    Eigen::Vector3i index;
    index.x() = std::min(std::max(static_cast<int>((delta_pos.x() - gl_low_.x()) / resolution_), 0), gl_size_.x());
    index.y() = std::min(std::max(static_cast<int>((delta_pos.y() - gl_low_.y()) / resolution_), 0), gl_size_.y());
    index.z() = std::min(std::max(static_cast<int>((delta_pos.z() - gl_low_.z()) / resolution_), 0), gl_size_.z());

    return index;
}

/**
 * @brief 将时间转换为索引
 *
 * @param time 当前时间
 *
 * @return 输入时间的索引
 */
int KinodynamicAstar::timeToIndex(double time) {
    int idx = static_cast<int>((time - start_time_) / time_resolution_);

    return idx;
}

/**
 * @brief 用于求解三次方程的根
 */
std::vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d) {
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0) {
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        dts.push_back(-a2 / 3 + (S + T));
        return dts;
    } else if (D == 0) {
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3 + S + S);
        dts.push_back(-a2 / 3 - S);
        return dts;
    } else {
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
        return dts;
    }
}

/**
 * @brief 使用四次方公式计算根
 *
 * @param a,b,c,d,e 四次方方程的系数
 *
 * @return 四次方程的根
 */
std::vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e) {
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double              y1 = ys.front();
    double              r  = a3 * a3 / 4 - a2 + y1;
    if (r < 0) {
        return dts;
    }

    double R = sqrt(r);
    double D, E;
    if (R != 0) {
        D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    } else {
        D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
        E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D)) {
        dts.push_back(-a3 / 4 + R / 2 + D / 2);
        dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E)) {
        dts.push_back(-a3 / 4 - R / 2 + E / 2);
        dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
}

/**
 * @brief 使用庞特里亚金最小原理估计从起点到终点的启发式数值
 *
 * @param start_pos 起点位置
 * @param start_vel 起点速度
 * @param end_pos 终点位置
 * @param end_vel 终点速度
 * @param optimal_time 最优时间
 *
 * @return 启发式距离
 */
double KinodynamicAstar::estimateHeuristic(Eigen::Vector3d start_pos,
                                           Eigen::Vector3d start_vel,
                                           Eigen::Vector3d end_pos,
                                           Eigen::Vector3d end_vel,
                                           double&         optimal_time) {
    const Eigen::Vector3d dp = end_pos - start_pos;  // 位置差
    const Eigen::Vector3d v0 = start_vel;            // 起始速度
    const Eigen::Vector3d v1 = end_vel;              // 终止速度

    double c1 = -36.0 * dp.dot(dp);
    double c2 = 24.0 * (v0 + v1).dot(dp);
    double c3 = -4.0 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
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
        double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;

        // 找到最小代价，并更新
        if (c < cost) {
            cost = c;
            t_d  = t;
        }
    }

    optimal_time = t_d;

    return (1.0 + tie_breaker_) * cost;
}

/**
 * @brief 根据起始状态、输入和持续时间，计算末端状态
 *
 * @param state 起始状态
 * @param um 控制输入
 * @param tau 持续时间
 *
 * @return 末端状态
 */
// Eigen::Matrix<double, 6, 1> KinodynamicAstar::stateTransit(const Eigen::Matrix<double, 6, 1> state,
//                                                            Eigen::Vector3d                   um,
//                                                            double                            tau) {
std::tuple<Eigen::Vector3d, Eigen::Vector3d> KinodynamicAstar::stateTransit(const Eigen::Vector3d& pos,
                                                                            const Eigen::Vector3d& vel,
                                                                            const Eigen::Vector3d& input,
                                                                            const double           t) {
    // // 状态转移矩阵
    // Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    // A(0, 3) = tau, A(1, 4) = tau, A(2, 5) = tau;
    // Eigen::Matrix<double, 6, 3> B = Eigen::Matrix<double, 6, 3>::Zero();
    // B(0, 0) = tau * tau / 2.0, B(1, 1) = tau * tau / 2.0, B(2, 2) = tau * tau / 2.0;
    // B(3, 0) = tau, B(4, 1) = tau, B(5, 2) = tau;

    // Eigen::Matrix<double, 6, 1> end_state = A * state + B * um;

    // Eigen::Matrix<double, 6, 1> end_state;
    // end_state(0) = state(0) + state(3) * tau + 0.5 * um(0) * tau * tau;
    // end_state(1) = state(1) + state(4) * tau + 0.5 * um(1) * tau * tau;
    // end_state(2) = state(2) + state(5) * tau + 0.5 * um(2) * tau * tau;
    // end_state(3) = end_state(3) + um(0) * tau;
    // end_state(4) = end_state(4) + um(1) * tau;
    // end_state(5) = end_state(5) + um(2) * tau;
    // return end_state;

    Eigen::Vector3d end_pos, end_vel;
    end_pos                                             = pos + t * vel + 0.5 * t * t * input;
    end_vel                                             = vel + t * input;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> pos_vel = std::pair<Eigen::Vector3d, Eigen::Vector3d>(end_pos, end_vel);
    return pos_vel;
}

/**
 * @brief 根据起始状态和终止状态，计算是否满足要求的路径
 */
bool KinodynamicAstar::computeShotTraj(const Eigen::Vector3d& start_pos,
                                       const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& end_pos,
                                       const Eigen::Vector3d& end_vel,
                                       double                 time_to_goal) {
    /* ---------- get coefficient ---------- */
    const Eigen::Vector3d p0  = start_pos;
    const Eigen::Vector3d dp  = end_pos - p0;
    const Eigen::Vector3d v0  = start_vel;
    const Eigen::Vector3d v1  = end_vel;
    const Eigen::Vector3d dv  = v1 - v0;
    double                t_d = time_to_goal;
    Eigen::MatrixXd       coef(3, 4);

    Eigen::Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
    Eigen::Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
    Eigen::Vector3d c = v0;
    Eigen::Vector3d d = p0;

    // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^3 + b*t^2 + v0*t + p0
    coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

    Eigen::Vector3d coord, vel, acc;
    Eigen::VectorXd poly1d, t, polyv, polya;
    Eigen::Vector3i index;

    Eigen::MatrixXd Tm(4, 4);
    Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

    /* ---------- Forward checking of trajectory ---------- */
    double t_delta = t_d / 10;
    for (double time = t_delta; time <= t_d; time += t_delta) {
        t = Eigen::VectorXd::Zero(4);
        for (int j = 0; j < 4; j++) {
            t(j) = pow(time, j);
        }

        // Check limitation of velocity and acceleration.
        for (int dim = 0; dim < 3; dim++) {
            poly1d     = coef.row(dim);
            coord(dim) = poly1d.dot(t);
            vel(dim)   = (Tm * poly1d).dot(t);
            acc(dim)   = (Tm * Tm * poly1d).dot(t);

            if (std::abs(vel(dim)) > max_vel_ || std::abs(acc(dim)) > max_acc_) {
                std::cout << "[Kino Astar] Over velocity or acceleration limits!\n";
                return false;
            }
        }
    }

    coef_shot_    = coef;
    t_shot_       = t_d;
    is_shot_succ_ = true;
    return true;
}

/**
 * @brief 获取搜索得到的路径
 */
// std::vector<Eigen::Vector3d> KinodynamicAstar::getPath(void) {
//     std::vector<Eigen::Vector3d> path;
//     KinoAstarNodePtr             current_ptr = terminate_ptr_;
//     while (current_ptr != nullptr) {
//         path.push_back(current_ptr->state.head(3));
//         current_ptr = current_ptr->parent;
//     }
//     std::reverse(path.begin(), path.end());

//     // 判断末端是否与实际的终点接近，如果不接近，则添加末端点
//     if ((path.back() - end_pos_).norm() > 0.1) {
//         path.push_back(end_pos_);
//     }
//     return path;
// }

/**
 * @brief 根据给定的时间间隔，获取路径
 */
std::vector<Eigen::Vector3d> KinodynamicAstar::getPath(double time_interval) {
    // // 首先获取节点的vector
    // std::vector<KinoAstarNodePtr> ptr_vec;
    // KinoAstarNodePtr              current_ptr = terminate_ptr_;
    // while (current_ptr != nullptr) {
    //     ptr_vec.push_back(current_ptr);
    //     current_ptr = current_ptr->parent;
    // }
    // std::reverse(ptr_vec.begin(), ptr_vec.end());

    // // 根据节点，获取对应的位置
    // std::vector<Eigen::Vector3d> path;
    // for (size_t i = 0; i < ptr_vec.size(); ++i) {
    //     current_ptr                               = ptr_vec[i];
    //     Eigen::Matrix<double, 6, 1> current_state = current_ptr->state;
    //     Eigen::Vector3d             input         = current_ptr->input;
    //     double                      duration      = current_ptr->duration;
    //     for (double t = 0.0; t <= duration; t += time_interval) {
    //         Eigen::Matrix<double, 6, 1> state = stateTransit(current_state, input, t);
    //         path.push_back(state.head(3));
    //     }
    // }

    std::vector<Eigen::Vector3d> path;

    // TEST:
    std::vector<KinoAstarNodePtr> node_path;

    KinoAstarNodePtr cur_ptr = terminate_ptr_, parent_ptr = cur_ptr->parent;

    // TEST:
    node_path.push_back(cur_ptr);

    // path.push_back(cur_ptr->pos);
    while (parent_ptr != nullptr) {
        Eigen::Vector3d pos, vel;
        pos                      = parent_ptr->pos;
        vel                      = parent_ptr->vel;
        Eigen::Vector3d input    = cur_ptr->input;
        double          duration = cur_ptr->duration;
        // std::cout << "Current duration: " << duration << std::endl;
        for (double t = duration; t >= time_interval; t -= time_interval) {
            auto [cur_pos, cur_vel] = stateTransit(pos, vel, input, t);
            path.push_back(cur_pos);
        }
        path.push_back(pos);

        // TEST:测试速度
        node_path.push_back(parent_ptr);

        // 更新
        cur_ptr    = cur_ptr->parent;
        parent_ptr = cur_ptr->parent;
    }
    std::reverse(path.begin(), path.end());

    // TEST:
    std::reverse(node_path.begin(), node_path.end());
    double tmp_time = node_path.front()->time;
    std::clog << "Start.\n";
    for (KinoAstarNodePtr ptr : node_path) {
        std::clog << "time: " << ptr->time - tmp_time << ", pos: " << ptr->pos.transpose()
                  << ", vel: " << ptr->vel.transpose() << std::endl;
    }
    std::clog << "End.\n";

    // 末尾情况
    if (is_shot_succ_) {
        Eigen::Vector3d coord;
        Eigen::VectorXd poly1d, time(4);

        for (double t = 0.0; t <= t_shot_; t += time_interval) {
            for (int j = 0; j < 4; j++)
                time(j) = pow(t, j);

            for (int dim = 0; dim < 3; dim++) {
                poly1d     = coef_shot_.row(dim);
                coord(dim) = poly1d.dot(time);
            }
            path.push_back(coord);
        }
    } else if ((path.back() - end_pos_).norm() > 0.1) {
        path.push_back(end_pos_);
    }
    return path;
}

/**
 * @brief
 * 根据给定的时间间隔，返回包含具体时间的轨迹。（1）根据节点与父节点的关系，获得对应时间间隔下每一段的采样点。（2）判断末尾是否需要求解更详细的轨迹段。
 */
std::vector<std::tuple<double, Eigen::Vector3d>> KinodynamicAstar::getTimePath(double time_interval) {
    std::vector<std::tuple<double, Eigen::Vector3d>> time_path;

    KinoAstarNodePtr cur_ptr = terminate_ptr_, parent_ptr = cur_ptr->parent;
    // path.push_back(cur_ptr->pos);
    while (parent_ptr != nullptr) {
        // 起始的位置、速度和时间
        Eigen::Vector3d pos, vel;
        pos       = parent_ptr->pos;
        vel       = parent_ptr->vel;
        double tm = parent_ptr->time;
        // 给定的控制输入和持续时间
        Eigen::Vector3d input    = cur_ptr->input;
        double          duration = cur_ptr->duration;
        for (double t = duration; t >= time_interval; t -= time_interval) {
            auto [cur_pos, cur_vel] = stateTransit(pos, vel, input, t);
            time_path.emplace_back(tm + t, cur_pos);
        }
        time_path.emplace_back(tm, pos);

        // 更新
        cur_ptr    = cur_ptr->parent;
        parent_ptr = cur_ptr->parent;
    }
    std::reverse(time_path.begin(), time_path.end());

    // 末尾情况
    if (is_shot_succ_) {
        Eigen::Vector3d coord;
        Eigen::VectorXd poly1d, time(4);

        for (double t = 0.0; t <= t_shot_; t += time_interval) {
            for (int j = 0; j < 4; j++)
                time(j) = pow(t, j);

            for (int dim = 0; dim < 3; dim++) {
                poly1d     = coef_shot_.row(dim);
                coord(dim) = poly1d.dot(time);
            }
            time_path.emplace_back(terminate_ptr_->time + t, coord);
        }
    } else if ((std::get<1>(time_path.back()) - end_pos_).norm() > 0.1) {
        time_path.emplace_back(std::get<0>(time_path.back()) + 1.0, end_pos_);
        // time_path.push_back(std::pair<double, Eigen::Vector3d>(time_path.back().first + 1.0, end_pos_));
    }
    return time_path;
}

/* 仅用于测试使用的函数 */
bool KinodynamicAstar::testOccupied(const Eigen::Vector3d& pos) {
    // 判断地图内部以及是否占用

    // 计算偏差
    const Eigen::Vector3d err_pos = pos - center_;

    // 在地图外部，默认没有占用
    if (err_pos.x() < gl_low_.x() || err_pos.y() < gl_low_.y() || err_pos.z() < gl_low_.z() || err_pos.x() > gl_upp_.x()
        || err_pos.y() > gl_upp_.y() || err_pos.z() > gl_upp_.z()) {
        return false;
    }

    // 判断占用情况
    const Eigen::Vector3i idx = ((err_pos - gl_low_) / resolution_).cast<int>();
    if (grid_map_[idx.x() * gl_size_.y() * gl_size_.z() + idx.y() * gl_size_.z() + idx.z()] == OCCUPIED) {
        return true;
    }
    return false;
}

// void KinodynamicAstar::setParam(ros::NodeHandle& nh) {
//     nh.param("search/map_size_x", map_size_3d_[0], 40.0);
//     nh.param("search/map_size_y", map_size_3d_[1], 40.0);
//     nh.param("search/map_size_z", map_size_3d_[2], 5.0);
//     nh.param("search/max_tau", max_tau_, -1.0);
//     nh.param("search/init_max_tau", init_max_tau_, -1.0);
//     nh.param("search/max_vel", max_vel_, -1.0);
//     nh.param("search/max_acc", max_acc_, -1.0);
//     nh.param("search/w_time", w_time_, -1.0);
//     nh.param("search/resolution_astar", resolution_, -1.0);
//     nh.param("search/time_resolution", time_resolution_, -1.0);
//     nh.param("search/lambda_heu", lambda_heu_, -1.0);
//     nh.param("search/allocate_num", allocate_num_, -1);
//     nh.param("search/check_num", check_num_, -1);
//     nh.param("search/optimistic", optimistic_, true);

//     origin_      = Eigen::Vector3d(-map_size_3d_[0] / 2.0, -map_size_3d_[1] / 2.0, 0.0);
//     tie_breaker_ = 1.0 + 1.0 / 10000;

//     double vel_margin;
//     nh.param("search/vel_margin", vel_margin, 0.0);
//     max_vel_ += vel_margin;

//     // FIXME:
//     test_kino_path_pub_ = nh.advertise<visualization_msgs::Marker>("test_kino_path", 1);
// }

// void KinodynamicAstar::retrievePath(KinoAstarNodePtr end_node) {
//     // Clear path_nodes_.
//     path_nodes_.clear();

//     // Set start node.
//     KinoAstarNodePtr cur_node = end_node;
//     path_nodes_.push_back(cur_node);

//     // Loop until find the first start node whose parent is nullptr.
//     while (cur_node->parent != nullptr) {
//         // if (cur_node->input.norm() > 10.0) {
//         //     std::cout << "Here: " << cur_node->input.transpose() << std::endl;
//         // }
//         // std::cout << "state: " << cur_node->state.transpose() << std::endl;
//         cur_node = cur_node->parent;
//         path_nodes_.push_back(cur_node);
//     }

//     // get the path from start to end
//     std::reverse(path_nodes_.begin(), path_nodes_.end());

//     // for (size_t i = 0; i < path_nodes_.size(); ++i) {
//     //     KinoAstarNodePtr node = path_nodes_[i];
//     //     std::cout << "retrievePath" << std::endl;
//     //     std::cout << "state: " << node->state.transpose() << std::endl;
//     // }
// }

// void KinodynamicAstar::init() {
//     /* ---------- map params ---------- */
//     cout << "origin_: " << origin_.transpose() << endl;
//     cout << "map size: " << map_size_3d_.transpose() << endl;

//     /* ---------- pre-allocated node ---------- */
//     path_node_pool_.resize(allocate_num_);
//     for (int i = 0; i < allocate_num_; i++) {
//         path_node_pool_[i] = std::make_shared<PathNode>();
//     }

//     phi_          = Eigen::MatrixXd::Identity(6, 6);
//     use_node_num_ = 0;
//     iter_num_     = 0;
// }

// std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t) {
//     std::vector<Eigen::Vector3d> state_list;

//     // /* ---------- get traj of searching ---------- */
//     // KinoAstarNodePtr                 node = path_nodes_.back();
//     // Eigen::Matrix<double, 6, 1> x0, xt;

//     // while (node->parent != nullptr) {
//     //     Eigen::Vector3d ut       = node->input;
//     //     double          duration = node->duration;
//     //     x0                       = node->parent->state;

//     //     for (double t = duration; t >= -1e-5; t -= delta_t) {
//     //         stateTransit(x0, xt, ut, t);
//     //         state_list.push_back(xt.head(3));
//     //     }
//     //     node = node->parent;
//     // }
//     // reverse(state_list.begin(), state_list.end());
//     /* Get trajectory of kino searching */
//     // std::cout << path_nodes_.size() << std::endl;
//     // std::cout << path_nodes_[0]->state.transpose() << std::endl;
//     // std::cout << path_nodes_[1]->state.transpose() << std::endl;
//     // std::cout << path_nodes_[2]->state.transpose() << std::endl;
//     // for (KinoAstarNodePtr node : path_nodes_) {

//     /* Get trajectory of kino path searching */
//     KinoAstarNodePtr node = path_nodes_.back();
//     while (node->parent != nullptr) {
//         // Input and duration
//         Eigen::Vector3d             input    = node->input;
//         double                      duration = node->duration;
//         Eigen::Matrix<double, 6, 1> x0       = node->parent->state, x1;

//         // Backward calculation.
//         for (double t = duration; t > 1e-3; t -= delta_t) {
//             stateTransit(x0, x1, input, duration);
//             state_list.push_back(x1.head(3));
//         }
//         node = node->parent;
//     }
//     // From start node to end node.
//     std::reverse(state_list.begin(), state_list.end());

//     /* ---------- get traj of one shot ---------- */
//     if (is_shot_succ_) {
//         Eigen::Vector3d coord;
//         Eigen::VectorXd poly1d, time(4);

//         for (double t = delta_t; t <= t_shot_; t += delta_t) {
//             for (int j = 0; j < 4; j++)
//                 time(j) = pow(t, j);

//             for (int dim = 0; dim < 3; dim++) {
//                 poly1d     = coef_shot_.row(dim);
//                 coord(dim) = poly1d.dot(time);
//             }
//             state_list.push_back(coord);
//         }
//     }

//     return state_list;
// }

// /**
//  * @brief Get inner points and corresponding time
//  *
//  * @param times Duration of every piece
//  * @param pts Point of every piece
//  * @param delta_t Abandon.
//  */
// void KinodynamicAstar::getMincoTrajParams(Eigen::VectorXd& times, Eigen::MatrixXd& pts, double delta_t) {
//     int row_num = path_nodes_.size();
//     times.resize(row_num - 1);
//     pts.resize(3, row_num);
//     for (size_t i = 0; i < path_nodes_.size(); ++i) {
//         KinoAstarNodePtr node = path_nodes_[i];
//         if (i > 0) {
//             times(i - 1) = node->duration;
//         }
//         pts.col(i) = node->state.head(3);
//     }

//     // pubTestKinoPath();
//     // for (KinoAstarNodePtr path_node : path_nodes_) {
//     //     std::cout << path_node->state.head(3).transpose() << ", time: " << path_node->time << std::endl;
//     // }
//     // int whole_piece = path_nodes_.size();
//     // // std::cout << "Whole piece: " << whole_piece << std::endl;

//     // if (whole_piece > 4) {
//     //     pts.resize(3, whole_piece - 2);
//     //     times.resize(whole_piece - 1);

//     //     for (int i = 0; i < whole_piece - 2; ++i) {
//     //         pts.col(i) = path_nodes_[i + 1]->state.head(3);
//     //         times[i]   = path_nodes_[i + 1]->time - path_nodes_[i]->time;
//     //     }
//     //     // int delta_piece = (int)std::ceil(((double)whole_piece - 1.0) / 3.0 + );
//     //     // std::cout << "Delta piece: " << delta_piece << std::endl;
//     //     // // int delta_piece = (whole_piece - 1) / 3 + 1;
//     //     // pts.resize(3, 2);
//     //     // times.resize(3);

//     //     // int last_piece = 0, idx = 0;
//     //     // for (int i = delta_piece; i < whole_piece; i += delta_piece) {
//     //     //     std::cout << i << std::endl;
//     //     //     pts.col(idx) = path_nodes_[i]->state.head(3);
//     //     //     times[idx]   = path_nodes_[i]->time - path_nodes_[last_piece]->time;
//     //     //     last_piece   = i;
//     //     //     ++idx;
//     //     // }
//     //     // times[idx] = path_nodes_.back()->time - path_nodes_[last_piece]->time;
//     //     // std::cout << "Calculation OK.\n";
//     // } else if (whole_piece == 4) {
//     //     pts.resize(3, 2);
//     //     times.resize(3);
//     //     int idx = 0;
//     //     for (int i = 1; i < whole_piece - 1; ++i) {
//     //         pts.col(idx) = path_nodes_[i]->state.head(3);
//     //         times[idx]   = path_nodes_[i]->time - path_nodes_[i - 1]->time;
//     //         ++idx;
//     //     }
//     //     times[idx] = path_nodes_[3]->time - path_nodes_[2]->time;
//     // } else if (whole_piece == 3) {
//     //     pts.resize(3, 1);
//     //     times.resize(2);
//     //     pts.col(0) = path_nodes_[1]->state.head(3);
//     //     times[0]   = path_nodes_[1]->time - path_nodes_[0]->time;
//     //     times[1]   = path_nodes_[2]->time - path_nodes_[1]->time;
//     // } else {
//     //     ROS_WARN("No more inner points! Maybe near the goal point.");
//     //     pts.resize(3, 1);
//     //     times.resize(2);
//     //     pts.col(0) = (path_nodes_.front()->state.head(3) + path_nodes_.back()->state.head(3)) / 2.0;
//     //     times.setConstant(path_nodes_.back()->time - path_nodes_.front()->time) / 2.0;
//     // }

//     // std::vector<Eigen::Vector3d> state_list;

//     // /* ---------- get traj of searching ---------- */
//     // KinoAstarNodePtr                 node = path_nodes_.back();
//     // Eigen::Matrix<double, 6, 1> x0, xt;

//     // double duration = 0.0;
//     // while (node->parent != nullptr) {
//     //     Eigen::Vector3d ut       = node->input;
//     //     double          duration = node->duration;
//     //     x0                       = node->parent->state;

//     //     for (double t = duration; t >= -1e-5; t -= delta_t) {
//     //         stateTransit(x0, xt, ut, t);
//     //         state_list.push_back(xt.head(3));
//     //     }
//     //     node = node->parent;
//     // }
//     // reverse(state_list.begin(), state_list.end());

//     // /* Convert state_list to Eigen::MatrixXd */
//     // pts.resize(3, state_list.size() - 2);  // Only calculate inner points.
//     // for (size_t i = 1; i < state_list.size() - 2; ++i) {
//     //     pts.col(i) = state_list[i];
//     // }
//     // times.resize(pts.cols());
//     // times.setConstant(delta_t);
// }

// void KinodynamicAstar::getSamples(double&                  ts,
//                                   vector<Eigen::Vector3d>& point_set,
//                                   vector<Eigen::Vector3d>& start_end_derivatives) {
//     /* ---------- path duration ---------- */
//     double T_sum = 0.0;
//     if (is_shot_succ_)
//         T_sum += t_shot_;
//     KinoAstarNodePtr node = path_nodes_.back();
//     while (node->parent != nullptr) {
//         T_sum += node->duration;
//         node = node->parent;
//     }
//     // cout << "duration:" << T_sum << endl;

//     // Calculate boundary vel and acc
//     Eigen::Vector3d end_vel, end_acc;
//     double          t;
//     if (is_shot_succ_) {
//         t       = t_shot_;
//         end_vel = end_vel_;
//         for (int dim = 0; dim < 3; ++dim) {
//             Eigen::Vector4d coe = coef_shot_.row(dim);
//             end_acc(dim)        = 2 * coe(2) + 6 * coe(3) * t_shot_;
//         }
//     } else {
//         t       = path_nodes_.back()->duration;
//         end_vel = node->state.tail(3);
//         end_acc = path_nodes_.back()->input;
//     }

//     // Get point samples
//     int seg_num           = floor(T_sum / ts);
//     seg_num               = max(8, seg_num);
//     ts                    = T_sum / double(seg_num);
//     bool sample_shot_traj = is_shot_succ_;
//     node                  = path_nodes_.back();

//     for (double ti = T_sum; ti > -1e-5; ti -= ts) {
//         if (sample_shot_traj) {
//             // samples on shot traj
//             Eigen::Vector3d coord;
//             Eigen::Vector4d poly1d, time;

//             for (int j = 0; j < 4; j++)
//                 time(j) = pow(t, j);

//             for (int dim = 0; dim < 3; dim++) {
//                 poly1d     = coef_shot_.row(dim);
//                 coord(dim) = poly1d.dot(time);
//             }

//             point_set.push_back(coord);
//             t -= ts;

//             /* end of segment */
//             if (t < -1e-5) {
//                 sample_shot_traj = false;
//                 if (node->parent != nullptr)
//                     t += node->duration;
//             }
//         } else {
//             // samples on searched traj
//             Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
//             Eigen::Matrix<double, 6, 1> xt;
//             Eigen::Vector3d             ut = node->input;

//             stateTransit(x0, xt, ut, t);

//             point_set.push_back(xt.head(3));
//             t -= ts;

//             // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
//             if (t < -1e-5 && node->parent->parent != nullptr) {
//                 node = node->parent;
//                 t += node->duration;
//             }
//         }
//     }
//     reverse(point_set.begin(), point_set.end());

//     // calculate start acc
//     Eigen::Vector3d start_acc;
//     if (path_nodes_.back()->parent == nullptr) {
//         // no searched traj, calculate by shot traj
//         start_acc = 2 * coef_shot_.col(2);
//     } else {
//         // input of searched traj
//         start_acc = node->input;
//     }

//     start_end_derivatives.push_back(start_vel_);
//     start_end_derivatives.push_back(end_vel);
//     start_end_derivatives.push_back(start_acc);
//     start_end_derivatives.push_back(end_acc);
// }

// std::vector<KinoAstarNodePtr> KinodynamicAstar::getVisitedNodes() {
//     vector<KinoAstarNodePtr> visited;
//     visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
//     return visited;
// }

// Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt) {
//     Eigen::Vector3i idx = ((pt - origin_) / resolution_).array().floor().cast<int>();

//     return idx;
// }

// /**
//  * @brief state0 to state1
//  *
//  * @param state0
//  * @param state1
//  * @param um input control
//  * @param tau time interval between state0 to state1
//  */
// void KinodynamicAstar::stateTransit(const Eigen::Matrix<double, 6, 1>& state0,
//                                     Eigen::Matrix<double, 6, 1>&       state1,
//                                     Eigen::Vector3d                    um,
//                                     double                             tau) {
//     for (int i = 0; i < 3; ++i) {
//         phi_(i, i + 3) = tau;
//     }

//     Eigen::Matrix<double, 6, 1> integral;
//     integral.head(3) = 0.5 * pow(tau, 2) * um;
//     integral.tail(3) = tau * um;

//     state1 = phi_ * state0 + integral;
// }

// /**
//  * @brief get kino path
//  *
//  * @return path
//  */
// std::vector<Eigen::Vector3d> KinodynamicAstar::getPath() {
//     std::vector<Eigen::Vector3d> path;
//     path.reserve(path_nodes_.size());

//     for (KinoAstarNodePtr node : path_nodes_) {
//         path.push_back(node->state.head(3));
//     }

//     return path;
// }

// /**
//  * @brief Publish kino path.
//  */
// void KinodynamicAstar::pubTestKinoPath() {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id    = "world";
//     marker.header.stamp       = ros::Time::now();
//     marker.ns                 = "points";
//     marker.id                 = 0;
//     marker.type               = visualization_msgs::Marker::POINTS;
//     marker.action             = visualization_msgs::Marker::ADD;
//     marker.pose.orientation.w = 1.0;
//     marker.scale.x            = 0.1;
//     marker.scale.y            = 0.1;
//     marker.scale.z            = 0.1;
//     marker.color.a            = 1.0;
//     marker.color.r            = 1.0;
//     marker.color.g            = 0.0;
//     marker.color.b            = 0.0;

//     std::vector<Eigen::Vector3d> tmp_path = getKinoTraj(0.5);
//     // std::vector<Eigen::Vector3d> tmp_path;
//     // KinoAstarNodePtr                  start_node = path_nodes_[0], end_node = path_nodes_[1];
//     // Eigen::Matrix<double, 6, 1>  start_state = start_node->state;
//     // Eigen::Vector3d              input       = start_node->input;
//     // double                       duration    = start_node->duration;
//     // double                       ts          = duration / 10.0;

//     // // std::cout << "start node: " << start_node->state.transpose() << std::endl;
//     // // std::cout << "end node: " << end_node->state.transpose() << std::endl;
//     // // std::cout << "input: " << um.transpose() << std::endl;

//     // for (double t = ts; t < duration; t += ts) {
//     //     // std::cout << "Current t: " << t << ", input: " << um.transpose() << std::endl;
//     //     Eigen::Vector3d state = stateTransit(start_state, input, t).head(3);
//     //     std::cout << "Time: " << t << ", State: " << state.transpose() << std::endl;
//     //     tmp_path.push_back(state);
//     // }

//     // // std::cout << "Tmp path size: " << tmp_path.size() << std::endl;

//     for (const Eigen::Vector3d& tmp_pt : tmp_path) {
//         // std::cout << tmp_pt.transpose() << std::endl;
//         geometry_msgs::Point pt;
//         pt.x = tmp_pt.x();
//         pt.y = tmp_pt.y();
//         pt.z = tmp_pt.z();
//         marker.points.push_back(pt);
//     }

//     test_kino_path_pub_.publish(marker);
// }
