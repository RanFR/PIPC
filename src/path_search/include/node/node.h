#pragma once

#include <Eigen/Core>
#include <limits>
#include <memory>
#include <unordered_map>

constexpr double inf = std::numeric_limits<double>::infinity();

class AstarNode;
using AstarNodePtr = std::shared_ptr<AstarNode>;

class KinoAstarNode;
using KinoAstarNodePtr = std::shared_ptr<KinoAstarNode>;

class DWANode;
using DWANodePtr = std::shared_ptr<DWANode>;

enum NodeState : char { OPENED, CLOSED, NONE };

enum GridState : char { FREE, OCCUPIED };

class AstarNode {
public:
    AstarNode() = default;
    AstarNode(int i, int j, int k) {
        index << i, j, k;
    }

    Eigen::Vector3i index;

    double g_score {inf};
    double f_score {inf};

    NodeState    node_state {NONE};
    AstarNodePtr parent {nullptr};
};

class KinoAstarNode {
public:
    /* -------------------- */
    Eigen::Vector3d pos, vel;  // 存储当前节点的位置和速度
    double          time;      // 当前当前节点的时间

    // Eigen::Matrix<double, 6, 1> state;  // 存储起始位置和速度
    // Eigen::Vector3i             index;  // 位置的索引

    // int    time_index;  // 当前时间的索引

    Eigen::Vector3d input;     // 父节点到当前节点的控制输入
    double          duration;  // 父节点到当前节点的持续时间

    double g_score, f_score;  // 节点的g值和f值

    KinoAstarNodePtr parent;  // 父节点

    NodeState node_state;  // 节点状态，开、闭和未知

    /* -------------------- */
    KinoAstarNode();
    ~KinoAstarNode() {};
};

class DWANode {
public:
    DWANode() = default;

    Eigen::Vector3d pos, vel, acc;  // 存储节点状态：位置、速度和加速度

    double time {0.0};  // 存储当前节点的时间

    double score {0.0};  // 用于存储当前节点的代价
    double cost {0.0};   //用于存储历史代价和当前节点代价

    DWANodePtr parent {nullptr};  // 父节点
};

class NodeComparator {
public:
    // Astar
    bool operator()(AstarNodePtr a, AstarNodePtr b) {
        return a->f_score > b->f_score;
    }

    // Kinodynamic Astar
    bool operator()(KinoAstarNodePtr node1, KinoAstarNodePtr node2) {
        return node1->f_score > node2->f_score;
    }

    // DWA
    bool operator()(DWANodePtr a, DWANodePtr b) const {
        return a->cost < b->cost;
    }
};

template<typename T> class MatrixHash : std::unary_function<T, size_t> {
public:
    size_t operator()(const T& matrix) const;
};

class NodeHashTable {
private:
    /* data */
    using HashMap = std::unordered_map<Eigen::Vector4i, KinoAstarNodePtr, MatrixHash<Eigen::Vector4i>>;
    HashMap data_4d_;  // Pos index and time index

public:
    NodeHashTable(/* args */) {}
    ~NodeHashTable() {}

    // 将位置索引和时间索引和对应的节点进行插入
    void insert(Eigen::Vector3i idx, int time_idx, KinoAstarNodePtr ptr);

    // 清除对应
    void erase(Eigen::Vector3i idx, int time_idx);

    // 查找对应关系
    KinoAstarNodePtr find(Eigen::Vector3i idx, int time_idx);

    // 清除data数据
    void clear();
};

/* DWA哈希表 */
class DWAHashTable {
public:
    DWAHashTable() = default;

    // 将哈希表中存储的所有节点指针存入vector中
    std::vector<DWANodePtr> convertToVector();

    // 根据位置索引和时间索引查找对应的节点指针
    DWANodePtr find(const int& idx);

    // 根据位置索引和时间索引将对应的节点指针插入哈希表中
    void insert(const int& idx, DWANodePtr ptr);

    // // 根据位置索引和时间索引清除对应的节点指针
    // void erase(const Eigen::Vector3i& pos_idx, int time_idx);

    // 重置哈希表数据
    void reset();

    // // 相同位置索引和时间索引情况下，更新节点指针
    // void update(const Eigen::Vector3i& pos_idx, int time_idx, DWANodePtr new_ptr);

private:
    std::unordered_map<int, DWANodePtr> hash_table_;
};
