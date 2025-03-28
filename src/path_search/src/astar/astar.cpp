#include "astar/astar.h"

/**
 * @param resolution 分辨率
 * @param gl_low 全局地图的最小坐标
 * @param gl_upp 全局地图的最大坐标
 */
void Astar::initMap(const double resolution, const Eigen::Vector3d& gl_low, const Eigen::Vector3d& gl_upp) {
    center_     = Eigen::Vector3d::Zero();
    resolution_ = resolution;
    gl_low_     = gl_low;
    gl_upp_     = gl_upp;
    gl_size_    = ((gl_upp - gl_low) / resolution_).cast<int>() + Eigen::Vector3i::Ones();
    // 预先定义内部栅格地图的大小，0表示没有占用，1表示占用
    grid_map_.resize(gl_size_.prod(), FREE);

    // 预先定义A*算法的节点
    astar_node_map_.resize(gl_size_.x());
    for (int i = 0; i < gl_size_.x(); ++i) {
        astar_node_map_[i].resize(gl_size_.y());
        for (int j = 0; j < gl_size_.y(); ++j) {
            astar_node_map_[i][j].resize(gl_size_.z());
            for (int k = 0; k < gl_size_.z(); ++k) {
                astar_node_map_[i][j][k] = std::make_shared<AstarNode>(i, j, k);
            }
        }
    }
}

/**
 * @brief 重置A星节点情况
 */
void Astar::reset(void) {
    for (int i = 0; i < gl_size_.x(); ++i) {
        for (int j = 0; j < gl_size_.y(); ++j) {
            for (int k = 0; k < gl_size_.z(); ++k) {
                astar_node_map_[i][j][k]->node_state = NodeState::NONE;
                astar_node_map_[i][j][k]->parent     = nullptr;
                astar_node_map_[i][j][k]->g_score    = inf;
                astar_node_map_[i][j][k]->f_score    = inf;
            }
        }
    }
}

/**
 * @brief 设定算法中心点
 *
 * @param center 给定中心点
 */
void Astar::setCenter(const Eigen::Vector3d& center) {
    center_ = center;
}

/**
 * @brief 设置障碍位置
 *
 * @param coord_x x坐标
 * @param coord_y y坐标
 * @param coord_z z坐标
 */
void Astar::setObs(double coord_x, double coord_y, double coord_z) {
    // 根据中心点计算A星地图中的位置
    coord_x -= center_.x();
    coord_y -= center_.y();
    coord_z -= center_.z();

    // 超出地图范围，直接返回，不设置障碍
    if (coord_x < gl_low_(0) || coord_y < gl_low_(1) || coord_z < gl_low_(2) || coord_x >= gl_upp_(0)
        || coord_y >= gl_upp_(1) || coord_z >= gl_upp_(2)) {
        return;
    }

    // 获得障碍在地图中的索引坐标
    const int x = static_cast<int>((coord_x - gl_low_(0)) / resolution_);
    const int y = static_cast<int>((coord_y - gl_low_(1)) / resolution_);
    const int z = static_cast<int>((coord_z - gl_low_(2)) / resolution_);

    grid_map_[x * gl_size_.y() * gl_size_.z() + y * gl_size_.z() + z] = OCCUPIED;
}

/**
 * @brief 根据输入的pcl地图设置障碍
 *
 * @param cloud pcl地图
 * @param radius 膨胀大小
 */
void Astar::setObsPcl(const pcl::PointCloud<pcl::PointXYZ>& cloud, double radius) {
    std::fill(grid_map_.begin(), grid_map_.end(), FREE);

    // 遍历输入的pcl地图
    for (const pcl::PointXYZ& pcl_pt : cloud) {
        // 转化为Eigen形式
        Eigen::Vector3d pt(pcl_pt.x, pcl_pt.y, pcl_pt.z);

        // 检查可行性，不可行则跳过该点
        if (!isPointFeasible(pt)) {
            continue;
        }

        // 进行膨胀，并条用setObs方法设置障碍
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
 * @brief 根据vector类型的地图，设置障碍
 *
 * @param cloud vector形式存储的障碍点
 * @param radius 膨胀半径
 */
void Astar::setObsVector(std::vector<Eigen::Vector3d>& cloud, double radius) {
    // 遍历给定障碍点地图的点
    for (const Eigen::Vector3d& pt : cloud) {
        // 给定点有效（在地图范围内，且不在障碍物中）
        if (!isPointFeasible(pt)) {
            // 无效，直接跳过
            continue;
        }

        // 膨胀，设置障碍
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
 * @brief 判断给定点是否在地图中
 *
 * @param pt 给定点
 * @return 在地图中返回true，否则返回false
 */
bool Astar::isInMap(const Eigen::Vector3d& pt) {
    // 计算距离中心点的偏差
    Eigen::Vector3d error = pt - center_;
    // 与给定的最大点和最小点坐标进行判断
    if (error.x() >= gl_low_.x() && error.x() < gl_upp_.x() && error.y() >= gl_low_.y() && error.y() < gl_upp_.y()
        && error.z() >= gl_low_.z() && error.z() < gl_upp_.z()) {
        // 如果在范围内，返回true
        return true;
    }
    return false;
}

/**
 * @brief 检查给定点pt可行性（检查是否在地图范围内，以及是否在障碍物内部）
 *
 * @param pt 给定点坐标
 * @return 如果可行返回true，否则返回false
 */
bool Astar::isPointFeasible(const Eigen::Vector3d& pt) {
    // 检查给定点pt是否在地图中
    if (!isInMap(pt)) {
        return false;
    }

    // 检查给定点占用情况
    if (isOccupied(coordToGridIndex(pt))) {
        return false;
    }

    // 在地图中，且未被占用，表明给定点不在障碍物内部
    return true;
}

/**
 * @brief 将栅格坐标转化为实际坐标
 *
 * @param index 栅格坐标
 * @return 实际坐标
 */
Eigen::Vector3d Astar::gridIndexToCoord(const Eigen::Vector3i& index) {
    Eigen::Vector3d pt {index.cast<double>() * resolution_ + gl_low_ + center_};
    // pt(0) = ((double)index(0) + 0.5) * resolution_ + gl_low_(0);
    // pt(1) = ((double)index(1) + 0.5) * resolution_ + gl_low_(1);
    // pt(2) = ((double)index(2) + 0.5) * resolution_ + gl_low_(2);
    // pt += center_;
    return pt;
}

/**
 * @brief 将实际坐标转化为栅格坐标
 *
 * @param pt 实际坐标
 * @return 栅格坐标
 */
Eigen::Vector3i Astar::coordToGridIndex(const Eigen::Vector3d& pt) {
    // 计算偏离点
    Eigen::Vector3d c_pt = pt - center_ - gl_low_;

    const int       x {static_cast<int>(c_pt.x() / resolution_)};
    const int       y {static_cast<int>(c_pt.y() / resolution_)};
    const int       z {static_cast<int>(c_pt.z() / resolution_)};
    Eigen::Vector3i index {x, y, z};

    return index;
}

/**
 * @brief 获取占据地图
 *
 * @param cloud 存储占据点的pcl
 */
void Astar::getOccupiedPcl(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    cloud.points.clear();
    for (int i = 0; i < gl_size_.x(); ++i) {
        for (int j = 0; j < gl_size_.y(); ++j) {
            for (int k = 0; k < gl_size_.z(); ++k) {
                if (isOccupied(i, j, k)) {
                    pcl::PointXYZ pt;
                    // 从double转化为float
                    pt.x = static_cast<float>(i * resolution_ + gl_low_.x() + center_.x());
                    pt.y = static_cast<float>(j * resolution_ + gl_low_.y() + center_.y());
                    pt.z = static_cast<float>(k * resolution_ + gl_low_.z() + center_.z());
                    // 存储到pcl中
                    cloud.points.push_back(pt);
                }
            }
        }
    }
}

/**
 * @brief 检查从p1到p2组成的线段，是否经过障碍物
 */
bool Astar::isLineFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    const Eigen::Vector3d vector    = p2 - p1;
    const int             point_num = static_cast<int>(vector.norm() / resolution_);

    // 根据设定的分辨率，遍历从p1点到p2点上的采样点，检查是否在地图中以及是否处于占据状态
    for (int i = 1; i <= point_num; ++i) {
        const Eigen::Vector3d coord = p1 + vector * i / (point_num + 1);
        if (!isPointFeasible(coord)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 检查给定的多个点组成的轨迹，是否可行
 *
 * @param path vector存储的多个点
 * @return 如果可行，返回true，否则返回false
 */
bool Astar::isPathFeasible(const std::vector<Eigen::Vector3d>& path) {
    if (path.size() == 0) {
        return true;
    }

    for (int i = 0; i < path.size() - 1; ++i) {
        const Eigen::Vector3d &p1 = path[i], &p2 = path[i + 1];

        if (!isLineFeasible(p1, p2)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 计算启发式函数值
 *
 * @param node1 起始节点
 * @param node2 终止节点
 * @return 返回从起始节点到终止节点的启发式函数值
 */
double Astar::calcHeu(AstarNodePtr node1, AstarNodePtr node2) {
    // 使用的启发式方法
    int    distance_norm = 3;  // 0:Euclidean 1:Manhattan 2:L_infty 3:Diagonal 4:Dijkstra
    double h             = 0.0;
    // 起点和终点
    Eigen::Vector3i start_index = node1->index;
    Eigen::Vector3i end_index   = node2->index;

    switch (distance_norm) {
        case 0:  // Euclidean
        {
            const Eigen::Vector3d delta = (start_index - end_index).cast<double>();
            h                           = delta.norm();
            break;
        }
        case 1:  // Manhattan
        {
            // 获得差值的绝对值向量
            Eigen::Vector3d delta = (end_index - start_index).cast<double>().array().abs();
            h                     = delta.sum();
            break;
        }
        case 2:  // L_infty，取差值的绝对值最大的一个
        {
            Eigen::Vector3d delta = (end_index - start_index).cast<double>().array().abs();
            h                     = delta.maxCoeff();
            break;
        }
        case 3:  // Diagonal
        {
            double distance[3];
            distance[0] = fabs((start_index(0) - end_index(0)));
            distance[1] = fabs((start_index(1) - end_index(1)));
            distance[2] = fabs((start_index(2) - end_index(2)));
            std::sort(distance, distance + 3);
            h = (std::sqrt(3.0) - std::sqrt(2.0)) * distance[0] + (std::sqrt(3.0) - 1) * distance[1] + distance[2];
            break;
        }
        default:
            break;
    }

    // Tie Breaker
    h = h * (1.0 + 1.0 / 10000);

    return h;
}

/**
 * @brief 获得当前点的邻居节点，以及对应的代价值
 *
 * @param cur 当前节点
 * @param neighbors 邻居节点
 * @param costs 代价值
 */
void Astar::getNeighbors(AstarNodePtr cur, std::vector<AstarNodePtr>& neighbors, std::vector<double>& costs) {
    // 清空容器，便于后续存储
    neighbors.clear();
    costs.clear();

    // 获得起始点索引
    int current_x = cur->index.x();
    int current_y = cur->index.y();
    int current_z = cur->index.z();

    // 对周围进行遍历
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                // 它本身
                if (i == 0 && j == 0 && k == 0) {
                    continue;
                }

                // 获得邻居索引
                int nx = current_x + i;
                int ny = current_y + j;
                int nz = current_z + k;

                // 超出地图范围，跳过
                if (nx < 0 || ny < 0 || nz < 0 || nx >= gl_size_(0) || ny >= gl_size_(1) || nz >= gl_size_(2)) {
                    continue;
                }

                // 在障碍物中，跳过
                if (isOccupied(nx, ny, nz)) {
                    continue;
                }

                // 插入当前邻居节点
                AstarNodePtr tmp_ptr = astar_node_map_[nx][ny][nz];
                neighbors.push_back(tmp_ptr);
                costs.push_back(std::sqrt(static_cast<double>(i * i + j * j + k * k)));
            }
        }
    }
}

/**
 * @brief 使用A星搜索从起点到终点的路径
 *
 * @param start_pt 起点
 * @param end_pt 终点
 * @return 路径是否存在，存在返回true，否则返回false
 */
bool Astar::searchPath(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) {
    start_pt_                 = start_pt;
    end_pt_                   = end_pt;
    Eigen::Vector3i start_idx = coordToGridIndex(start_pt);
    Eigen::Vector3i end_idx   = coordToGridIndex(end_pt);

    std::cerr << "1";

    // 检查起点和终点是否占据，如果被占据，设置一个搜索相邻的起点或者终点
    if (isOccupied(start_idx)) {
        const int POINT_NUM {8};
        double    r {resolution_};
        ROS_INFO("\033[41;37m [AStar] Start point in obstacle.\033[0m");
        while (true) {
            bool flag = false;
            for (int i = 0; i < POINT_NUM; ++i) {
                Eigen::Vector3d pt;
                pt << start_pt.x() + r * std::sin(PI * 2.0 * static_cast<double>(i) / static_cast<double>(POINT_NUM)),
                    start_pt.y() + r * std::cos(PI * 2.0 * static_cast<double>(i) / static_cast<double>(POINT_NUM)),
                    start_pt.z();

                if (!isOccupied(coordToGridIndex(pt))) {
                    start_pt_ = pt;
                    start_idx = coordToGridIndex(start_pt_);
                    flag      = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start goal! (%f %f %f)\033[0m",
                         start_pt_.x(),
                         start_pt_.y(),
                         start_pt_.z());
                break;
            }
            r += resolution_;
        }
    }
    if (isOccupied(end_idx)) {
        const int POINT_NUM {8};
        double    r {resolution_};
        ROS_INFO("\033[41;37m [AStar] End point in obstacle.\033[0m");
        while (true) {
            bool flag = false;
            for (int i = 0; i < POINT_NUM; ++i) {
                Eigen::Vector3d pt;
                pt << end_pt.x() + r * std::sin(PI * 2.0 * static_cast<double>(i) / static_cast<double>(POINT_NUM)),
                    end_pt.y() + r * std::cos(PI * 2.0 * static_cast<double>(i) / static_cast<double>(POINT_NUM)),
                    end_pt.z();

                if (!isOccupied(coordToGridIndex(pt))) {
                    end_pt_ = pt;
                    end_idx = coordToGridIndex(end_pt_);
                    flag    = true;
                }
            }
            if (flag) {
                ROS_INFO("\033[1;32m Change start goal! (%f %f %f)\033[0m", end_pt_.x(), end_pt_.y(), end_pt_.z());
                break;
            }
            r += resolution_;
        }
    }

    std::cerr << "2";

    AstarNodePtr start_ptr = astar_node_map_[start_idx(0)][start_idx(1)][start_idx(2)];

    std::cerr << "3\n";
    std::cerr << end_idx.transpose();
    std::cerr << ", " << gl_size_.transpose() << std::endl;
    AstarNodePtr end_ptr = astar_node_map_[end_idx(0)][end_idx(1)][end_idx(2)];
    std::cerr << "4";
    start_ptr->node_state = OPENED;
    start_ptr->parent     = nullptr;
    start_ptr->g_score    = 0.0;
    start_ptr->f_score    = calcHeu(start_ptr, end_ptr);

    // 初始化开集
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, NodeComparator> empty_lists;
    open_lists_.swap(empty_lists);
    open_lists_.push(start_ptr);

    AstarNodePtr              current_ptr  = nullptr;
    AstarNodePtr              neighbor_ptr = nullptr;
    std::vector<AstarNodePtr> neighbors;
    std::vector<double>       costs;

    std::cerr << "ok.";

    // 持续遍历开集
    while (!open_lists_.empty()) {
        current_ptr             = open_lists_.top();
        current_ptr->node_state = CLOSED;  // add current node in close_list
        open_lists_.pop();

        // 如果当前点与终点索引相同，认为到达终点
        if (current_ptr->index == end_idx) {
            terminate_ptr_ = current_ptr;
            return true;
        }

        // 获得当前节点的邻居节点和对应的代价值
        getNeighbors(current_ptr, neighbors, costs);  // find neighbors and their costs

        // 遍历邻居
        for (int i = 0; i < neighbors.size(); i++) {
            neighbor_ptr = neighbors[i];
            double gh    = current_ptr->g_score + costs[i];
            double fh    = gh + calcHeu(neighbor_ptr, end_ptr);

            // 没有被查询过，则加入开集
            if (neighbor_ptr->node_state == NONE) {
                neighbor_ptr->node_state = OPENED;  // add in open list
                neighbor_ptr->g_score    = gh;
                neighbor_ptr->f_score    = fh;
                neighbor_ptr->parent     = current_ptr;
                open_lists_.push(neighbor_ptr);
            } else if (neighbor_ptr->node_state == OPENED) {  // 在开集中，则检查是否具有更优的代价
                if (neighbor_ptr->g_score > gh) {
                    // 更新开集代价
                    neighbor_ptr->g_score = gh;
                    neighbor_ptr->f_score = fh;
                    neighbor_ptr->parent  = current_ptr;
                }
            } else {  // 已经在闭集中，直接跳过
                continue;
            }
        }
    }

    // 开集遍历完成，但是并没有找到路径
    return false;
}

/**
 * @brief 根据之前搜索的A*路径，返回对应的路径
 *
 * @param path 返回的A*路径
 */
void Astar::getPath(std::vector<Eigen::Vector3d>& path) {
    // 重置路径
    path.clear();

    // 首先获得节点组成的路径
    std::vector<AstarNodePtr> node_path;
    AstarNodePtr              tmp = terminate_ptr_;
    while (tmp->parent != nullptr) {
        node_path.push_back(tmp);
        tmp = tmp->parent;
    }

    // 根据获得的节点路径，将对应的索引转换为实际坐标，组成最终路径
    for (AstarNodePtr ptr : node_path) {
        Eigen::Vector3d coord = gridIndexToCoord(ptr->index);
        path.push_back(coord);
    }
    path.push_back(start_pt_);  // 加入起点

    // 反向，获得从起点到终点的路径
    std::reverse(path.begin(), path.end());
    path.push_back(end_pt_);  // 加入终点
}

std::vector<Eigen::Vector3d> Astar::getPath() {
    // 重置路径
    std::vector<Eigen::Vector3d> path;

    // 首先获得节点组成的路径
    std::vector<AstarNodePtr> node_path;
    AstarNodePtr              tmp = terminate_ptr_;
    while (tmp->parent != nullptr) {
        node_path.push_back(tmp);
        tmp = tmp->parent;
    }

    // 根据获得的节点路径，将对应的索引转换为实际坐标，组成最终路径
    for (AstarNodePtr ptr : node_path) {
        Eigen::Vector3d coord = gridIndexToCoord(ptr->index);
        path.push_back(coord);
    }
    path.push_back(start_pt_);  // 加入起点

    // 反向，获得从起点到终点的路径
    std::reverse(path.begin(), path.end());
    path.push_back(end_pt_);  // 加入终点\

    return path;
}

/**
 * @brief 简化A星路径
 *
 * @param astar_path 等待简化的A星路径
 * @param waypoint 简化后的路径
 */
void Astar::simplifyPath(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint) {
    // A*路径只有一个点，直接返回，不需要简化
    if (astar_path.size() <= 1) {
        return;
    }

    // 放入起点
    waypoint.push_back(astar_path[0]);
    // 如果A星只有2个点，再放入第二个点，然后结束返回
    if (astar_path.size() <= 2) {
        waypoint.push_back(astar_path[1]);
        return;
    }

    Eigen::Vector3d vec_last = astar_path[1] - astar_path[0];
    for (int i = 2; i < astar_path.size(); i++) {
        Eigen::Vector3d vec = astar_path[i] - astar_path[i - 1];
        // 只有上一段轨迹和下一段轨迹是同方向，进行结合，省略中间点
        if (vec.dot(vec_last) == vec.norm() * vec_last.norm()) {
            // The included angle of two vectors is 0
            continue;
        }
        waypoint.push_back(astar_path[i - 1]);
        vec_last = vec;
    }
    // 放入最后一个点
    waypoint.push_back(astar_path[astar_path.size() - 1]);
}

/**
 * @brief 使用floydHandle方法简化路径
 *
 * @param astar_path 输入的A星路径
 * @param waypoint 输出路径
 */
void Astar::floydHandle(const std::vector<Eigen::Vector3d>& astar_path, std::vector<Eigen::Vector3d>& waypoint) {
    // 首先简化路径，删除多余点
    waypoint.clear();
    simplifyPath(astar_path, waypoint);

    // 生成Floyd路径，运行两次是为了最优化路径
    // generate Floyd path(twice for optimal trajectory)
    for (int time = 0; time < 2; ++time) {
        // 从最后一个点开始遍历
        for (int i = waypoint.size() - 1; i > 0; --i) {
            // 从起点到当前的终点
            for (int j = 0; j < i - 1; j++) {
                // 检查镇个线段是否可行
                if (isLineFeasible(waypoint[i], waypoint[j])) {
                    // 如果可行就删除中间的点，进行简化
                    for (int k = i - 1; k > j; k--) {
                        waypoint.erase(waypoint.begin() + k);  // delete redundant inflection points
                    }
                    i = j;
                    break;
                }
            }
        }
    }
}

/** 私有函数 **/

/**
 * @brief 检查给定点占据情况
 *
 * @return 如果处于占据状态，返回true，否则返回false
 */
bool Astar::isOccupied(int idx_x, int idx_y, int idx_z) {
    // 如果在地图外部，认为没有处于占据状态，或者设定的状态就是非占用
    // 满足点在地图内部，同时对应的栅格地图为占据状态，返回true
    return (idx_x >= 0 && idx_x < gl_size_(0) && idx_y >= 0 && idx_y < gl_size_(1) && idx_z >= 0 && idx_z < gl_size_(2)
            && (grid_map_[idx_x * gl_size_(1) * gl_size_(2) + idx_y * gl_size_(2) + idx_z] == OCCUPIED));
}

bool Astar::isOccupied(const Eigen::Vector3i index) {
    return isOccupied(index(0), index(1), index(2));
}

bool Astar::isFree(int idx_x, int idx_y, int idx_z) {
    return (idx_x >= 0 && idx_x < gl_size_(0) && idx_y >= 0 && idx_y < gl_size_(1) && idx_z >= 0 && idx_z < gl_size_(2)
            && (grid_map_[idx_x * gl_size_(1) * gl_size_(2) + idx_y * gl_size_(2) + idx_z] != OCCUPIED));
}

bool Astar::isFree(const Eigen::Vector3i index) {
    return isFree(index(0), index(1), index(2));
}
