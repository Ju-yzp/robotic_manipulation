#ifndef FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_
#define FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_

// cpp
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <queue>
#include <stack>
#include <thread>
#include <vector>

// eigen
#include <Eigen/Eigen>

namespace fast_motion_planning {

template <typename T, uint32_t Cap>
class FixedMemoryPool {
public:
    FixedMemoryPool() {
        capacity_ = (Cap << 1) - (Cap >> 1);  // 预申请1.5倍容量
        memory_ptr_ = new T[capacity_];
        for (uint32_t id{0}; id < capacity_; ++id) id_stack_.emplace(id);
    }

    // 对象析构时，释放申请内存
    ~FixedMemoryPool() { delete[] memory_ptr_; }

    // 分配内存
    T* allocate() {
        while (flag.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
        if (id_stack_.empty()) return nullptr;

        uint32_t offset = id_stack_.top();
        id_stack_.pop();
        flag.clear(std::memory_order_release);
        return memory_ptr_ + offset;
    }

    void free(void* object) {
        T* ptr = (T*)object;
        uint32_t offset = ptr - memory_ptr_;
        while (flag.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
        if (offset < capacity_) id_stack_.emplace(offset);
        flag.clear(std::memory_order_release);
    }

private:
    // 指向申请内存块的指针
    T* memory_ptr_;

    // 原子标志
    std::atomic_flag flag = ATOMIC_FLAG_INIT;

    // 相对于内存起始地址的偏移量，也就是索引
    std::stack<uint32_t> id_stack_;

    // 申请的内存容量
    uint32_t capacity_{0};
};

template <typename Data, size_t Dim, uint32_t Cap>
class NearestNeighbor {
public:
    using PointType = Eigen::Vector<double, Dim>;
    using DataVector = std::vector<Data>;

private:
    struct KdTreeNode {
        uint8_t div_axis;                  // 分割轴
        uint32_t tree_size{1};             // 以该节点作为根节点的树的节点数量
        Data data;                         // 数据，一定有point成员(PointType类型)
        KdTreeNode* parent{nullptr};       // 父节点
        KdTreeNode* right_child{nullptr};  // 右孩子节点
        KdTreeNode* left_child{nullptr};   // 左孩子节点
        float alpha_bal;                   // 平衡系数
        double radius{0.0};                // 从节点张成的球型空间
    };

    // 两个点在每个轴的差必须都大于这个值，否则认为是同一个点
    static constexpr const double ESP = 1e-2;

    // 失去平衡性的树的节点数目如果少于规定最小值，就会在主线程内串行重构建
    static constexpr const int min_unbalanced_tree_size_ = 5;

    // 如果失平衡的树的节点数目大于这个数目就在后台线程进行重构建工作
    static constexpr const int mutil_rebuild_point_num_ = 30;

    // 正在进行替换和读取操作
    int REBUILDING_FLAG;

    // 衡量节点平衡的系数
    double balance_criterion_param_{0.7};

    // 搜索计数器
    std::atomic<int> search_state_counter_{0};

    std::atomic_flag rebuild_flag_ = ATOMIC_FLAG_INIT;

    // 重建互斥锁
    std::mutex rebuild_mutex;

    // 后台线程是否退出标志
    std::atomic<bool> termination_flag_{false};

    // 待插入的节点队列,配合rebuid_fmutex_进行多线程保护
    std::queue<Data> pending_queue_;

    using CmpFunc = std::function<bool(Data, Data)>;

    // 比较函数容器
    std::vector<CmpFunc> cfv_;

    // 后台重构建线程
    std::thread rebuild_thread;

    // 根节点
    KdTreeNode* root_{nullptr};

    // 固定大小内存池
    FixedMemoryPool<KdTreeNode, Cap> fmp_;

    // 待重建的树的根节点
    KdTreeNode** rebuild_tree_{nullptr};

    // 用于后台线程存储重建树时的数据
    DataVector rebuild_pcl_storage_;

    // 用于主线程存储重建树节点数据
    DataVector pcl_storage_;

    // 释放以root作为树的根节点的整颗树的内存，非线程安全
    void freeTree(KdTreeNode* root) {
        if (!root) return;

        std::stack<KdTreeNode*> ns;
        ns.emplace(root);

        KdTreeNode* node{nullptr};
        while (!ns.empty()) {
            node = ns.top();
            ns.pop();
            if (node->right_child) ns.emplace(node->right_child);
            if (node->left_child) ns.emplace(node->left_child);
            fmp_.free(node);
        }
    }

    // 记录构建新树的信息载体
    struct MessageStorage {
        KdTreeNode* node;
        int32_t start{0}, end{0};
    };

    // 用于构建新树
    std::optional<std::vector<MessageStorage>> buildTree(
        KdTreeNode** root, int32_t start, int32_t end, DataVector& data_set) {
        if (start > end) return std::nullopt;
        int32_t mid = (start + end) >> 1;

        if (!(*root)) *root = fmp_.allocate();
        double min_value[Dim], max_value[Dim], dim_range[Dim];
        std::fill(min_value, min_value + Dim, std::numeric_limits<double>::infinity());
        std::fill(max_value, max_value + Dim, std::numeric_limits<double>::lowest());
        std::fill(dim_range, dim_range + Dim, 0);

        // 获取各个维度上的极值
        for (int32_t i{start}; i < end + 1; ++i) {
            for (size_t j{0}; j < Dim; ++j) {
                min_value[j] = std::min(data_set[i].point(j), min_value[j]);
                max_value[j] = std::max(data_set[i].point(j), max_value[j]);
            }
        }

        KdTreeNode* node = *root;
        for (size_t i{0}; i < Dim; ++i) dim_range[i] = max_value[i] - min_value[i];

        uint8_t div_axis = 0;

        for (size_t i{0}; i < Dim; ++i)
            div_axis = dim_range[i] > dim_range[div_axis] ? i : div_axis;

        node->tree_size = end - start + 1;

        std::nth_element(
            std::begin(data_set) + start, std::begin(data_set) + mid,
            std::begin(data_set) + end + 1, cfv_[div_axis]);

        node->data = data_set[mid];
        double max_dist_sqr = 0.0;
        for (int32_t i = start; i <= end; ++i) {
            double dist_sqr = calc_dist(data_set[i].point, node->data.point);
            if (dist_sqr > max_dist_sqr) {
                max_dist_sqr = dist_sqr;
            }
        }
        node->radius = max_dist_sqr;

        std::vector<MessageStorage> msv;
        MessageStorage ms;

        if (start > mid - 1 && mid + 1 > end) return std::nullopt;

        if (start <= mid - 1) {
            node->left_child = fmp_.allocate();
            ms.node = node->left_child;
            ms.start = start;
            ms.end = mid - 1;
            ms.node->parent = node;
            msv.emplace_back(ms);
        }
        if (mid + 1 <= end) {
            node->right_child = fmp_.allocate();
            ms.node = node->right_child;
            ms.start = mid + 1;
            ms.end = end;
            ms.node->parent = node;
            msv.emplace_back(ms);
        }

        return msv;
    }

    // 传入数据集，然后构建一颗以root为根节点的树,root指向的对象内容需要是一个空指针
    void buildTree(KdTreeNode** root, DataVector& data_set) {
        std::stack<MessageStorage> mss;

        std::optional<std::vector<MessageStorage>> msv;
        msv = buildTree(root, 0, uint32_t(data_set.size() - 1), data_set);

        if (msv.has_value()) {
            for (auto ms : msv.value()) {
                mss.push(ms);
            }
        }

        MessageStorage ms;
        while (!mss.empty()) {
            ms = mss.top();
            mss.pop();
            msv = buildTree(&ms.node, ms.start, ms.end, data_set);
            if (msv.has_value()) {
                for (auto ms : msv.value()) mss.push(ms);
            }
        }
    }

    // 从该叶子节点出发，回溯至整颗kd-tree的根节点,更新全部被影响的节点
    void update(KdTreeNode* node, bool allow_rebuild, int diff) {
        if (!node) return;

        KdTreeNode* parent = node->parent;
        KdTreeNode** rt{nullptr};
        while (parent) {
            parent->tree_size += diff;
            if (calc_dist(parent->data.point, node->data.point) > parent->radius)
                parent->radius = calc_dist(parent->data.point, node->data.point);
            if (allow_rebuild && criterionCheck(parent) &&
                parent->tree_size > min_unbalanced_tree_size_)
                rt = &parent;
            parent = parent->parent;
        }

        if (rt && *rt) rebuild(rt);
    }

    // 添加点至kd-tree中
    void add_point(Data data, KdTreeNode* root, bool allow_rebuild = true) {
        if (!root) return;

        KdTreeNode* parent = root;
        KdTreeNode** node{nullptr};
        while (parent) {
            if (rebuild_tree_ && *rebuild_tree_ == parent) {
                while (rebuild_flag_.test_and_set(std::memory_order_acquire))
                    std::this_thread::yield();
                pending_queue_.push(data);
                rebuild_flag_.clear(std::memory_order_release);
                return;
            } else {
                if (cfv_[parent->div_axis](parent->data, data)) {
                    if (parent->right_child) {
                        parent = parent->right_child;
                        continue;
                    }
                    node = &parent->right_child;
                    break;
                } else {
                    if (parent->left_child) {
                        parent = parent->left_child;
                        continue;
                    }
                    node = &parent->left_child;
                    break;
                }
            }
        }

        // 申请内存
        *node = fmp_.allocate();
        (*node)->data = data;
        (*node)->div_axis = (parent->div_axis + 1) % Dim;
        (*node)->parent = parent;
        update(*node, allow_rebuild, 1);

        // std::cout<<int(root_->tree_size)<<std::endl;
    }

    void mutilRebuild() {
        while (!termination_flag_.load(std::memory_order_acquire)) {
            std::unique_lock<std::mutex> rebuild_lock(rebuild_mutex);
            if (rebuild_tree_ && *rebuild_tree_) {
                while (rebuild_flag_.test_and_set(std::memory_order_acquire)) continue;
                std::cout << "wow" << std::endl;
                KdTreeNode* parent = (*rebuild_tree_)->parent;  // 记录父节点
                KdTreeNode* node{nullptr};
                KdTreeNode** new_root_node = &node;  // 记录新的点
                KdTreeNode* old_root_node = *rebuild_tree_;
                uint32_t old_tree_size = (*rebuild_tree_)->tree_size;
                flatten(old_root_node, rebuild_pcl_storage_);
                buildTree(new_root_node, rebuild_pcl_storage_);
                int num{0};
                while (!pending_queue_.empty()) {
                    Data data = pending_queue_.front();
                    pending_queue_.pop();
                    ++num;
                    add_point(data, *new_root_node, false);
                    rebuild_flag_.clear(std::memory_order_release);
                    if (num % 40 == 0) std::this_thread::sleep_for(std::chrono::microseconds(70));
                    while (rebuild_flag_.test_and_set(std::memory_order_acquire))
                        ;
                }
                rebuild_flag_.clear(std::memory_order_release);

                // 首先自旋等待没有搜索最邻近点，等待没有的时候,用新树替换掉旧树，并释放旧树申请的内存
                int expected_counter = 0;
                while (true) {
                    if (search_state_counter_.compare_exchange_strong(
                            expected_counter, -1, std::memory_order_acquire))
                        break;

                    // 避免太多空转，造成cpu利用率下降
                    expected_counter = 0;
                    std::this_thread::yield();
                }

                // 判断属于父节点的右子节点还是左子节点，然后使得指向正确的内存，并赋值子节点指向正确的父节点
                if (parent) {
                    if (parent->right_child == old_root_node) parent->right_child = *new_root_node;
                    if (parent->left_child == old_root_node) parent->left_child = *new_root_node;
                    (*new_root_node)->parent = parent;
                }
                search_state_counter_.store(0, std::memory_order_release);
                update(*new_root_node, true, (*new_root_node)->tree_size - old_tree_size);
                rebuild_tree_ = nullptr;

                rebuild_lock.unlock();
                freeTree(old_root_node);
                std::cout << "Finish rebuild tree" << std::endl;
            }

            // 休眠一段时间
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "Rebuild thread normally return" << std::endl;
    }

    void flatten(KdTreeNode* root, DataVector& storage) {
        storage.clear();  // 清空容器，防止旧数据干扰

        if (!root) return;
        storage.emplace_back(root->data);
        std::stack<KdTreeNode*> ns;
        ns.emplace(root);
        KdTreeNode* node{nullptr};
        while (!ns.empty()) {
            node = ns.top();
            ns.pop();
            if (node->right_child) ns.emplace(node->right_child);
            if (node->left_child) ns.emplace(node->left_child);
        }
    }

    // 寻找最近的点
    void search(PointType point, Data& nearest_node) {
        std::stack<KdTreeNode*> ns;
        nearest_node = root_->data;
        double nearest_dist = calc_dist(nearest_node.point, point);
        if (root_->right_child) ns.emplace(root_->right_child);
        if (root_->left_child) ns.emplace(root_->left_child);

        KdTreeNode* rebuilding_tree{nullptr};
        while (!ns.empty()) {
            KdTreeNode* node = ns.top();
            ns.pop();
            if (rebuild_tree_ && *rebuild_tree_ == node) {
                rebuilding_tree = *rebuild_tree_;
                continue;
            }
            double current_dist = calc_dist(node->data.point, point);

            if (current_dist < nearest_dist) {
                nearest_node = node->data;
                nearest_dist = current_dist;
            }

            if (current_dist - node->radius > nearest_dist) continue;

            if (node->right_child) ns.emplace(node->right_child);
            if (node->left_child) ns.emplace(node->left_child);
        }

        // 在serach_state_counter为-1时，不能进行搜索，但是serach_state_counter非0时，就会进行自增操作
        while (true && rebuilding_tree) {
            if (search_state_counter_.compare_exchange_strong(
                    REBUILDING_FLAG, 0, std::memory_order_acq_rel, std::memory_order_acquire))
                continue;
            else {
                std::cout << "wow" << std::endl;
                search_state_counter_.fetch_add(1, std::memory_order_release);
                search_by_tree(rebuilding_tree, point, nearest_node, nearest_dist);
                search_state_counter_.fetch_sub(1, std::memory_order_release);
                break;
            }
            std::this_thread::yield();
        }
    }

    // 搜索以root为根节点的树
    void search_by_tree(
        KdTreeNode* root, PointType point, Data& nearest_node, double nearest_dist) {
        std::stack<KdTreeNode*> ns;
        ns.emplace(root);

        while (!ns.empty()) {
            KdTreeNode* node = ns.top();
            ns.pop();
            double current_dist = calc_dist(node->data.point, point);

            if (current_dist < nearest_dist) {
                nearest_node = node->data;
                nearest_dist = current_dist;
            }

            if (current_dist - node->radius > nearest_dist) continue;

            if (node->right_child) ns.emplace(node->right_child);
            if (node->left_child) ns.emplace(node->left_child);
        }
    }

    // 计算两个点的平方距离
    double calc_dist(PointType a, PointType b) { return (a - b).norm(); }

    // 判断两个点是否相同
    bool isSamePoint(PointType a, PointType b) {
        for (size_t i{0}; i < Dim; ++i) {
            if (std::abs(a(i) - b(i)) > ESP) return false;
        }
        return true;
    }

    // 检查节点的平衡性
    bool criterionCheck(KdTreeNode* root) {
        if (root->tree_size <= min_unbalanced_tree_size_) return false;

        double balance_evaluation = 0.0f;
        KdTreeNode* child_ptr{nullptr};
        if (root->left_child)
            child_ptr = root->left_child;
        else if (root->right_child)
            child_ptr = root->right_child;
        else
            return false;

        balance_evaluation = double(child_ptr->tree_size) / double(root->tree_size);

        if (balance_evaluation > balance_criterion_param_ ||
            (1.0 - balance_evaluation) < balance_criterion_param_)
            return true;
        return false;
    }

    // 启动后台重构建线程
    void startThread() { rebuild_thread = std::thread(&NearestNeighbor::mutilRebuild, this); }

    void rebuild(KdTreeNode** root) {
        KdTreeNode* parent;
        if ((*root)->tree_size >= mutil_rebuild_point_num_) {
            // 拿不到锁就选择等待下一次节点更新时触发重建
            std::unique_lock<std::mutex> rebuild_lock(rebuild_mutex, std::try_to_lock);
            if (rebuild_lock.owns_lock()) {
                if (rebuild_tree_ && *rebuild_tree_ &&
                    (*rebuild_tree_)->tree_size < (*root)->tree_size)
                    rebuild_tree_ = root;
                if (!rebuild_tree_) rebuild_tree_ = root;
            }
        } else {
            KdTreeNode* old_root_node = *root;     // 记录旧树的根节点指向的内存位置
            pcl_storage_.clear();                  // 清理容器
            flatten(old_root_node, pcl_storage_);  // 收集旧树所有节点信息
            buildTree(root, pcl_storage_);         // 构建旧树
            freeTree(old_root_node);               // 释放旧树内存
        }
    }

public:
    NearestNeighbor() {
        // 初始化比较函数
        cfv_.resize(Dim);
        for (size_t id{0}; id < Dim; ++id)
            cfv_[id] = [id](Data a, Data b) { return a.point(id) < b.point(id); };

        REBUILDING_FLAG = -1;

        startThread();
    }

    // 调用析构函数时，释放申请的内存
    ~NearestNeighbor() {
        termination_flag_.store(true, std::memory_order_release);
        if (rebuild_thread.joinable()) rebuild_thread.join();
        freeTree(root_);
    }

    // 插入数据
    void insert(Data data) { add_point(data, root_); }

    // 构建
    void build(DataVector& data_set) { buildTree(&root_, data_set); }

    void reset() {
        freeTree(root_);
        root_ = nullptr;
    }

    // 搜索最邻近的节点
    void searchNearest(PointType point, Data& nearest_node) { search(point, nearest_node); }
};
}  // namespace fast_motion_planning

#endif
