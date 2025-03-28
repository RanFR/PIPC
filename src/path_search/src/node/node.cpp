#include "node/node.h"

KinoAstarNode::KinoAstarNode() {
    // 初始化位置和速度
    Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
    pos = vel = zero3d;

    // 初始化时间
    time = 0.0;

    // 初始化控制输入和持续时间
    input    = zero3d;
    duration = 0.0;

    // 初始化g值和f值
    g_score = f_score = inf;

    // 防止父节点指针悬空
    parent = nullptr;

    // 初始化节点状态
    node_state = NONE;

    // 原来参数
    // state.setZero();
    // g_score = inf;
    // f_score = inf;
    // input.setZero();
    // duration   = 0.0;
    // parent     = nullptr;
    // node_state = NONE;
}

template<typename T> size_t MatrixHash<T>::operator()(const T& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
}

/**
 * @brief 将给定的位置索引、时间索引与节点，插入到data数据集中
 */
void NodeHashTable::insert(Eigen::Vector3i idx, int time_idx, KinoAstarNodePtr ptr) {
    // 首先尝试在无序映射表中查找对应的映射值
    auto iter = data_4d_.find(Eigen::Vector4i(idx.x(), idx.y(), idx.z(), time_idx));

    // 如果对应索引为映射表末尾（不存在对应的映射），则插入对应的映射
    if (iter == data_4d_.end()) {
        data_4d_.insert(std::make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), ptr));
    }
}

/**
 * @brief 根据给定的位置索引和时间索引，删除对应的数据集中的映射
 */
void NodeHashTable::erase(Eigen::Vector3i idx, int time_idx) {
    // 试图查找对应的节点
    HashMap::iterator iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));

    // 如果找到节点（即并非数据集的末尾），进行清除
    if (iter != data_4d_.end()) {
        data_4d_.erase(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    }
}

/**
 * @brief 根据给定的位置索引和时间索引，搜索对应的节点信息，并返回
 *
 * @return 返回Kino节点，如果没有茶找到，则返回nullptr
 */
KinoAstarNodePtr NodeHashTable::find(Eigen::Vector3i idx, int time_idx) {
    HashMap::iterator iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    // Return the corresponding KinoAstarNode.
    return iter == data_4d_.end() ? nullptr : iter->second;
}

/**
 * @brief 清除哈希表
 */
void NodeHashTable::clear() {
    data_4d_.clear();
}

/* DWA哈希表 */

/**
 * @brief 将哈希表中的指针存入vector中
 */
std::vector<DWANodePtr> DWAHashTable::convertToVector() {
    std::vector<DWANodePtr> ptr_vector;

    for (auto& iter : hash_table_) {
        ptr_vector.push_back(iter.second);
    }

    return ptr_vector;
}

/**
 * @brief 根据给定的位置索引和时间索引，搜索对应的节点指针，并返回
 */
DWANodePtr DWAHashTable::find(const int& idx) {
    auto iter = hash_table_.find(idx);

    // 返回对应的节点指针，如果哈希表中不存在，返回nullptr
    return iter == hash_table_.end() ? nullptr : iter->second;
}

/* DWA哈希表相关函数 */
/**
 * @brief 根据位置索引和时间索引，将节点指针插入哈希表中
 *
 * @attention 必须在函数执行前确认对应索引在哈希表中不存在
 */
void DWAHashTable::insert(const int& idx, DWANodePtr ptr) {
    // 根据索引在插入节点指针
    hash_table_.insert(std::make_pair(idx, ptr));
}

/**
 * @brief 重置哈希表
 */
void DWAHashTable::reset() {
    std::unordered_map<int, DWANodePtr> empty_table;
    std::swap(empty_table, hash_table_);
}

// /**
//  * @brief 根据新给定的指针，更新哈希表
//  */
// void DWAHashTable::update(const Eigen::Vector3i& pos_idx, int time_idx, DWANodePtr new_ptr) {
//     // 查找哈希表索引
//     const Eigen::Vector4i IDX(pos_idx.x(), pos_idx.y(), pos_idx.z(), time_idx);
//     auto                  iter = hash_table_.find(IDX);

//     // 更新哈希表
//     iter->second = new_ptr;
// }
