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
#include <unordered_map>
#include <vector>

// eigen
#include <Eigen/Eigen>

namespace fast_motion_planning {

template <typename DataType>
class FixedMemoryPool {
public:
    explicit FixedMemoryPool(uint32_t max_capacity) {
        capacity_ = (max_capacity << 1) - (max_capacity >> 1);
        resource_ptr_ = new DataType[capacity_];
        for (uint32_t offset{0}; offset < capacity_; ++offset) offset_stack_.emplace(offset);
    }

    ~FixedMemoryPool() { delete[] resource_ptr_; }

    // 分配内存
    DataType* allocate() {
        while (flag_.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
        if (offset_stack_.empty()) throw "No enough memory to offer (offset_stack_ is empty)";

        uint32_t offset = offset_stack_.top();
        offset_stack_.pop();
        flag_.clear(std::memory_order_release);
        return resource_ptr_ + offset;
    }

    // 释放内存
    void free(DataType* ptr) {
        while (flag_.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
        uint32_t offset = ptr - resource_ptr_;
        if (offset < capacity_) offset_stack_.emplace(offset);
        flag_.clear(std::memory_order_release);
    }

private:
    // 固定内存池容量
    uint32_t capacity_;

    // 指向申请内存资源的指针
    DataType* resource_ptr_;

    // 存放偏移量的栈
    std::stack<uint32_t> offset_stack_;

    // 保护内存资源的原子标志
    std::atomic_flag flag_ = ATOMIC_FLAG_INIT;
};

template <typename DataType, size_t Dim>
class NearestNeighbor {
public:
    using PointType = Eigen::Vector<double, Dim>;
    using DataVector = std::vector<DataType>;

private:
    struct KdTreeNode {
        uint8_t div_axis{0};    // 分裂轴
        uint32_t tree_size{1};  // 以该节点作为根节点的树的节点数量
        DataType data;
        double radius{0.0};                // 以该节点作为根节点的树张成的球形空间
        KdTreeNode* parent{nullptr};       // 父节点
        KdTreeNode* left_child{nullptr};   // 左孩子节点
        KdTreeNode* right_child{nullptr};  // 右孩子节点
    };

    using CmpFunc = std::function<bool(DataType, DataType)>;

    // 不平衡树重建的最低节点数量要求
    static constexpr const int Min_Unbalanced_Tree_Size_ = 10;

    // 不平衡树在后台线程进行重建的最少节点数量要求
    static constexpr const int Mutil_Rebuild_Point_Num_ = 30;

    // 操作队列标志
    std::atomic_flag operation_flag_ = ATOMIC_FLAG_INIT;

    // 主线程进行最邻近搜索标志
    std::atomic_flag search_flag_ = ATOMIC_FLAG_INIT;

    // 重建树指针
    KdTreeNode** rebuild_tree_{nullptr};

    // 保护重建树
    std::mutex rebuild_mutex_;

    // 固定大小内存池，减少频繁的系统调用开销
    FixedMemoryPool<KdTreeNode> fmp_;

    // 缓存队列
    std::queue<DataType> pending_queue_;

    // 后台重构建线程
    std::thread rebuild_thread_;

    // 后台线程是否退出标志
    std::atomic<bool> termination_flag_{false};

    // 主线程重构建收集树节点信息容器
    DataVector pcl_storage_;

    // 后台重构建线程收集节点信息容器
    DataVector rebuild_pcl_storage_;

    // 根节点
    KdTreeNode* root_{nullptr};

    // 比较函数容器映射表
    std::unordered_map<uint8_t, CmpFunc> cmp_func_map_;

    // 不平衡系数
    double balance_criterion_param_{0.7};

    //
    KdTreeNode* temp_node_{nullptr};
    KdTreeNode* temp_rebuild_{nullptr};

    void flatten(KdTreeNode* root, DataVector& storage) {
        storage.clear();

        if (!root) return;
        std::stack<KdTreeNode*> ns;
        ns.emplace(root);
        KdTreeNode* node{nullptr};
        while (!ns.empty()) {
            node = ns.top();
            ns.pop();
            storage.emplace_back(node->data);
            if (node->right_child) ns.emplace(node->right_child);
            if (node->left_child) ns.emplace(node->left_child);
        }
    }

    // 释放以root为根节点的树的内存资源
    void freeTree(KdTreeNode* root) {
        std::stack<KdTreeNode*> ns;
        ns.emplace(root);

        KdTreeNode* node{nullptr};
        while (!ns.empty()) {
            node = ns.top();
            ns.pop();

            if (node->left_child) ns.emplace(node->left_child);
            if (node->right_child) ns.emplace(node->right_child);
            fmp_.free(node);
        }
    }

    void searchByTree(
        KdTreeNode* root, PointType point, DataType& nearest_node, double nearest_dist) {
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

    void update(KdTreeNode* node, bool allow_rebuild, int increase_size) {
        KdTreeNode** need_rebuild_tree{nullptr};
        PointType point = node->data.point;

        temp_node_ = nullptr;
        node = node->parent;
        while (node) {
            node->tree_size += increase_size;
            double dist = calc_dist(node->data.point, point);
            if (dist > node->radius) node->radius = dist;
            if (allow_rebuild && criterionCheck(node) &&
                node->tree_size > Min_Unbalanced_Tree_Size_) {
                temp_node_ = node;
            }
            node = node->parent;
            std::cout << "wo" << std::endl;
        }

        need_rebuild_tree = &temp_node_;
        if (*need_rebuild_tree) {
            if (*need_rebuild_tree == root_) {
                std::cout << "wow" << std::endl;
                rebuild(&root_);
            } else
                rebuild(need_rebuild_tree);
            need_rebuild_tree = nullptr;
        }
    }

    // 记录构建新树的信息载体
    struct MessageStorage {
        KdTreeNode* node;
        int32_t start{0}, end{0};
    };

    // 构建子树
    std::optional<std::vector<MessageStorage>> buildTree(
        KdTreeNode** root, int32_t start, int32_t end, DataVector& points) {
        if (start > end) return std::nullopt;
        int32_t mid = (start + end) >> 1;

        if (!(*root)) {
            *root = fmp_.allocate();
            std::cout << "w" << std::endl;
        }

        double min_value[Dim], max_value[Dim], dim_range[Dim];
        std::fill(min_value, min_value + Dim, std::numeric_limits<double>::infinity());
        std::fill(max_value, max_value + Dim, std::numeric_limits<double>::lowest());
        std::fill(dim_range, dim_range + Dim, 0);

        // 获取各个维度上的极值
        for (int32_t i{start}; i < end + 1; ++i) {
            for (size_t j{0}; j < Dim; ++j) {
                min_value[j] = std::min(points[i].point(j), min_value[j]);
                max_value[j] = std::max(points[i].point(j), max_value[j]);
            }
        }

        KdTreeNode* node = *root;
        for (size_t i{0}; i < Dim; ++i) dim_range[i] = max_value[i] - min_value[i];

        uint8_t div_axis = 0;

        for (size_t i{0}; i < Dim; ++i)
            div_axis = dim_range[i] > dim_range[div_axis] ? i : div_axis;

        node->tree_size = end - start + 1;
        node->div_axis = div_axis;

        std::nth_element(
            std::begin(points) + start, std::begin(points) + mid, std::begin(points) + end + 1,
            cmp_func_map_[div_axis]);

        node->data = points[mid];
        double max_dist_sqr = 0.0;
        for (int32_t i = start; i <= end; ++i) {
            double dist_sqr = calc_dist(points[i].point, node->data.point);
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

    void buildTree(KdTreeNode** root, DataVector& points) {
        std::stack<MessageStorage> mss;

        std::optional<std::vector<MessageStorage>> msv;
        msv = buildTree(root, 0, uint32_t(points.size() - 1), points);

        if (msv.has_value()) {
            for (auto ms : msv.value()) {
                mss.push(ms);
            }
        }

        MessageStorage ms;
        while (!mss.empty()) {
            ms = mss.top();
            mss.pop();
            msv = buildTree(&ms.node, ms.start, ms.end, points);
            if (msv.has_value()) {
                for (auto ms : msv.value()) mss.push(ms);
            }
        }
    }

    void mutilThread() {
        while (!termination_flag_.load(std::memory_order_acquire)) {
            std::unique_lock<std::mutex> rebuild_lock(rebuild_mutex_);
            if (rebuild_tree_) {
                // std::cout << "Start rebuild tree in multi-thread " << std::endl;
                // while (operation_flag_.test_and_set(std::memory_order_acquire))
                //     std::this_thread::yield();

                // // 保存旧树信息以及重建新树
                // KdTreeNode* parent_node = (*rebuild_tree_)->parent;
                // KdTreeNode* old_root_node = (*rebuild_tree_);
                // rebuild_pcl_storage_.clear();
                // KdTreeNode* new_root_node{nullptr};
                // flatten((*rebuild_tree_), rebuild_pcl_storage_);
                // buildTree(&new_root_node, rebuild_pcl_storage_);

                // // 将缓存队列中的点，插入新树中
                // int num{0};
                // while (true) {
                //     if (pending_queue_.empty()) break;
                //     DataType point = pending_queue_.front();
                //     pending_queue_.pop();
                //     add_by_point(point, new_root_node, false);
                //     operation_flag_.clear(std::memory_order_release);
                //     if (num % 40 == 0)
                //     std::this_thread::sleep_for(std::chrono::microseconds(200)); while
                //     (operation_flag_.test_and_set(std::memory_order_acquire))
                //         std::this_thread::yield();
                // }
                // operation_flag_.clear(std::memory_order_release);

                // // 新树替换旧树
                // while (search_flag_.test_and_set(std::memory_order_acquire))
                //     std::this_thread::yield();
                // if (parent_node) {
                //     new_root_node->parent = parent_node;
                //     if (&parent_node->right_child == rebuild_tree_)
                //         parent_node->right_child = new_root_node;
                //     if (&parent_node->left_child == rebuild_tree_)
                //         parent_node->left_child = new_root_node;
                //     new_root_node->parent = parent_node;
                // }
                // search_flag_.clear(std::memory_order_release);
                // rebuild_tree_ = nullptr;
                // rebuild_lock.unlock();
                // freeTree(old_root_node);
                // std::cout << "Finish rebuilding tree " << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
    }

    void start_thread() { rebuild_thread_ = std::thread(&NearestNeighbor::mutilThread, this); }

    // 返回两点之间的欧氏距离
    double calc_dist(PointType p1, PointType p2) { return (p1 - p2).norm(); }

    void rebuild(KdTreeNode** need_rebuild_tree) {
        if ((*need_rebuild_tree)->tree_size >= Mutil_Rebuild_Point_Num_) {
            std::cout << "!" << std::endl;
            std::unique_lock<std::mutex> rebuild_lock(rebuild_mutex_, std::try_to_lock);
            if (rebuild_lock.owns_lock()) {
                if (!rebuild_tree_ ||
                    ((*rebuild_tree_)->tree_size < (*need_rebuild_tree)->tree_size)) {
                    temp_rebuild_ = temp_node_;
                    rebuild_tree_ = &temp_rebuild_;
                }
            }
        } else {
            std::cout << "Start rebuild in main thread" << std::endl;
            KdTreeNode* parent_node = (*need_rebuild_tree)->parent;
            KdTreeNode* old_root_node = (*need_rebuild_tree);
            KdTreeNode* new_root_node{nullptr};
            flatten(old_root_node, pcl_storage_);
            buildTree(&new_root_node, pcl_storage_);
            std::cout << new_root_node->tree_size << std::endl;
            if (parent_node) {
                std::cout << "go " << std::endl;
                if (parent_node->left_child == old_root_node)
                    parent_node->left_child = new_root_node;
                else
                    parent_node->right_child = new_root_node;
            } else
                root_ = new_root_node;

            freeTree(old_root_node);
        }
    }

    // 检查节点的平衡性
    bool criterionCheck(KdTreeNode* node) {
        if (node->tree_size <= Min_Unbalanced_Tree_Size_) return false;

        double balance_evaluation{0.0};
        KdTreeNode* child_ptr = node->left_child ? node->left_child : node->right_child;
        if (!child_ptr) return false;

        balance_evaluation = double(child_ptr->tree_size) / double(node->tree_size);
        if (balance_evaluation > balance_criterion_param_ ||
            (1.0 - balance_evaluation) < balance_criterion_param_)
            return true;
        return false;
    }

    void add_by_point(DataType point, KdTreeNode* root, bool allow_rebuild = true) {
        KdTreeNode* parent = root;
        KdTreeNode** node{nullptr};
        while (parent) {
            if (rebuild_tree_ && *rebuild_tree_ == parent) {
                while (operation_flag_.test_and_set(std::memory_order_acquire))
                    std::this_thread::yield();
                pending_queue_.push(point);
                operation_flag_.clear(std::memory_order_release);
                return;
            } else {
                std::cout << "!!" << std::endl;
                if (cmp_func_map_[parent->div_axis](parent->data, point)) {
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

        std::cout << "size " << int(root_->tree_size) << std::endl;
        // 申请内存
        (*node) = fmp_.allocate();
        (*node)->data = point;
        (*node)->div_axis = (parent->div_axis + 1) % Dim;
        (*node)->parent = parent;
        update(*node, allow_rebuild, 1);
    }

public:
    NearestNeighbor(uint32_t max_capacity, double balance_criterion_param = 0.7)
        : fmp_(max_capacity), balance_criterion_param_(balance_criterion_param) {
        for (uint8_t id{0}; id < Dim; ++id)
            cmp_func_map_[id] = [id](DataType a, DataType b) { return a.point(id) < b.point(id); };
        // start_thread();
    }

    ~NearestNeighbor() {
        std::cout << int(root_->tree_size) << std::endl;
        termination_flag_.store(true, std::memory_order_release);
        if (rebuild_thread_.joinable()) rebuild_thread_.join();
        if (root_) freeTree(root_);
    }

    // 搜索最邻近点
    void search(PointType point, DataType& nearest_node) {
        std::stack<KdTreeNode*> ns;
        if (!root_) return;
        std::cout << "root size " << int(root_->tree_size) << std::endl;
        nearest_node = root_->data;
        double nearest_dist = calc_dist(nearest_node.point, point);
        if (root_->right_child) ns.emplace(root_->right_child);
        if (root_->left_child) ns.emplace(root_->left_child);

        KdTreeNode* rebuilding_tree{nullptr};
        KdTreeNode* node{nullptr};
        while (!ns.empty()) {
            node = ns.top();
            ns.pop();

            // 如果需要遍历正在重建的树,那么就优先遍历其他树先
            if (rebuild_tree_ && (*rebuild_tree_) == node) {
                rebuilding_tree = node;
                continue;
            }

            double cur_dist = calc_dist(node->data.point, nearest_node.point);
            if (cur_dist < nearest_dist) {
                nearest_node = node->data;
                nearest_dist = cur_dist;
            }

            if (cur_dist - node->radius > nearest_dist) continue;

            if (node->right_child) ns.emplace(node->right_child);
            if (node->left_child) ns.emplace(node->left_child);
        }

        if (rebuilding_tree) {
            while (search_flag_.test_and_set(std::memory_order_acquire)) std::this_thread::yield();
            searchByTree(rebuilding_tree, point, nearest_node, nearest_dist);
            search_flag_.clear(std::memory_order_release);
        }
    }

    void build(DataVector& points) { buildTree(&root_, points); }

    void add_point(DataType point) { add_by_point(point, root_, true); }

    // void iter(){
    //     std::stack<KdTreeNode *>ns;
    //     if (!root_) return ;
    //     ns.emplace(root_);
    //     KdTreeNode* node{nullptr};
    //     int count{0};
    //     while (!ns.empty()) {
    //         node = ns.top();
    //         ns.pop();
    //         //std::cout<<++count<<std::endl;
    //         if (node->right_child) ns.emplace(node->right_child);
    //         if (node->left_child) ns.emplace(node->left_child);
    //     }
    // }
};
}  // namespace fast_motion_planning

#endif
