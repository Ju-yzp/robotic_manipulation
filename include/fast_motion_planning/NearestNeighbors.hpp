#ifndef FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_
#define FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_

// cpp
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <queue>
#include <stack>
#include <thread>

// eigen
#include <Eigen/Eigen>


namespace fast_motion_planning {

template <typename T, uint32_t Cap>
class MemoryPool {
public:
    MemoryPool() {
        for (uint32_t i{0}; i < Cap; ++i) stack_.push(i);

        ptr_ = new T[Cap];
    }

    ~MemoryPool() { delete[] ptr_; }

    T* allocate() {
        while (flag.test_and_set(std::memory_order_acquire))
            ;
        if (stack_.empty()) return nullptr;
        uint32_t offset = stack_.top();
        stack_.pop();
        flag.clear(std::memory_order_release);
        return ptr_ + offset;
    }

    void free(void* ptr) {
        T* new_ptr = (T*)ptr;
        if (!new_ptr) return;
        uint32_t offset = new_ptr - ptr_;
        while (flag.test_and_set(std::memory_order_acquire))
            ;
        stack_.emplace(offset);
        flag.clear(std::memory_order_release);
    }

private:
    T* ptr_;
    std::stack<uint32_t, std::vector<uint32_t>> stack_;
    std::atomic_flag flag = ATOMIC_FLAG_INIT;
};

// 单线程版本
template <size_t dim>
class NearestNeighbors {  // TODO：下面全部使用的是欧氏距离
public:
    using PointType = Eigen::Vector<double, dim>;
    using PointVector = std::vector<PointType>;

private:
    static constexpr int minimal_unbalanced_tree_size = 10;
    static constexpr int multi_thread_rebuild_point_num = 1500;

    int searching_counter_ = {0};

    struct KdTreeNode {
        KdTreeNode() {
            for (int i{0}; i < dim; ++i) {
                node_range(i, 0) = std::numeric_limits<double>::infinity();
                node_range(i, 1) = std::numeric_limits<double>::lowest();
            }
        }
        KdTreeNode* parent{nullptr};               // 父节点
        KdTreeNode* right_child{nullptr};          // 右孩子节点
        KdTreeNode* left_child{nullptr};           // 左孩子节点
        PointType data = PointType::Zero();        // 数据
        uint8_t div_axis = 0;                      // 分割轴
        float alpha_bal;                           // 平衡系数
        Eigen::Matrix<double, dim, 2> node_range;  // 树所代表的空间
        uint32_t tree_size{1};  // TODO:最坏的情况，树的节点数目大于uint32_t所能表示的最大正整数
        std::atomic<bool> working_flag_{false};
    };

    using CmpFunc = std::function<bool(PointType, PointType)>;

    // 比较函数
    std::vector<CmpFunc> cmp_funv_;

    // 平衡系数
    float balance_criterion_param_{0.7};

    // 平衡中间数
    float alpha_bal_tmp_{0.5};

    float balance_criterion_param = 0.7f;

    KdTreeNode* root_{nullptr};

    MemoryPool<KdTreeNode, 100000> mp_;

    // 存储需要重构建的树的点集
    PointVector pcl_storage_, rebuild_pcl_storage_;
    // 重建的树
    KdTreeNode** rebuildTree_{nullptr};

    // 更新和搜索的锁，由于在rrt中，搜索最邻近点和添加新的节点是严重同步的，所以只需要一把锁就行
    std::mutex updtae_search_flag_;

    // 缓存队列
    std::queue<PointType> point_chache_;

    // 用于保护把缓存队列中的点添加至重构建树的过程
    std::mutex operation_mutex_;

    // 重构建线程停止标志
    bool termination_flag_{false};

    // 多线程重构建树的标志
    std::atomic<bool> rebuild_flag_{false};

    // 搜索的标志
    std::atomic<int> search_counter_{0};

    // 后台重构建进程
    std::thread rebuild_thread_;

    struct PointTypeCmp {
        PointType point;
        double dist{0.0};

        constexpr static double ESP = 1e-10;

        PointTypeCmp(PointType p = PointType(), float d = std::numeric_limits<double>::infinity())
            : point(p), dist(d) {}

        bool operator<(const PointTypeCmp& a) const {
            if (fabs(dist - a.dist) < ESP) return point(0) < a.point(0);
            return dist < a.dist;
        }
    };

    // 根堆
    struct ManualHeap {
    public:
        ManualHeap(int max_capacity = 100) {
            capacity = max_capacity;
            heap = new PointTypeCmp[capacity];
            heap_size = 0;
        }

        ~ManualHeap() { delete[] heap; }

        PointTypeCmp top() { return heap[0]; }

        void pop() {
            if (heap_size == 0) return;
            heap[0] = heap[heap_size - 1];
            --heap_size;
            moveDown(0);
            return;
        }

        void push(PointTypeCmp point) {
            if (heap_size >= capacity) return;
            heap[heap_size] = point;
            floatUp(heap_size);
            ++heap_size;
            return;
        }

        int size() { return heap_size; }

        void clear() { heap_size = 0; }

    private:
        int heap_size{0};
        int capacity{0};
        PointTypeCmp* heap;

        void moveDown(int heap_index) {
            int l = heap_index * 2 + 1;
            PointTypeCmp tmp = heap[heap_index];
            while (l < heap_size) {
                if (l + 1 < heap_size && heap[l] < heap[l + 1]) l++;
                if (tmp < heap[l]) {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                } else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }

        void floatUp(int heap_index) {
            int ancestor = (heap_index - 1) / 2;
            PointTypeCmp tmp = heap[heap_index];
            while (heap_index > 0) {
                if (heap[ancestor] < tmp) {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                } else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
    };

    struct MessageStorage {
        KdTreeNode* node;
        int32_t start, end;
    };

    std::optional<std::vector<MessageStorage>> buildTree(
        KdTreeNode** root, int32_t start, int32_t end, PointVector& points) {
        if (start > end) return std::nullopt;
        int32_t mid = (start + end) >> 1;

        if (!(*root)) *root = mp_.allocate();
        KdTreeNode* node = (*root);

        uint8_t div_axis{0};
        double min_value[dim], max_value[dim], dim_range[dim];
        std::fill(min_value, min_value + dim, std::numeric_limits<double>::infinity());
        std::fill(max_value, max_value + dim, std::numeric_limits<double>::lowest());
        std::fill(dim_range, dim_range + dim, 0);

        for (int32_t i{start}; i < end + 1; ++i) {
            for (size_t j{0}; j < dim; ++j) {
                min_value[j] = std::min(points[i][j], min_value[j]);
                max_value[j] = std::max(points[i][j], max_value[j]);
            }
        }

        for (size_t i{0}; i < dim; ++i) dim_range[i] = max_value[i] - min_value[i];

        for (size_t i{0}; i < dim; ++i) {
            div_axis = dim_range[i] > dim_range[div_axis] ? i : div_axis;
            node->node_range(i, 0) = min_value[i];
            node->node_range(i, 1) = max_value[i];
        }

        node->tree_size = end - start + 1;

        std::nth_element(
            std::begin(points) + start, std::begin(points) + mid, std::begin(points) + end + 1,
            cmp_funv_[div_axis]);

        node->data = points[mid];

        std::vector<MessageStorage> msv;
        MessageStorage ms;

        if (start > mid - 1 && mid + 1 > end) return std::nullopt;

        if (start <= mid - 1) {
            node->left_child = mp_.allocate();
            ms.node = node->left_child;
            ms.start = start;
            ms.end = mid - 1;
            ms.node->parent = node;
            msv.emplace_back(ms);
        }
        if (mid + 1 <= end) {
            node->right_child = mp_.allocate();
            ms.node = node->right_child;
            ms.start = mid + 1;
            ms.end = end;
            ms.node->parent = node;
            msv.emplace_back(ms);
        }

        return msv;
    }

    void update(KdTreeNode** node) {
        KdTreeNode* n = *node;
        KdTreeNode* parent = n->parent;
        if (parent) parent->tree_size += n->tree_size;
    }

    void multiThreadRebuild() {
        bool terminated{false};
        KdTreeNode *parent{nullptr}, **new_node_ptr{nullptr};
        static int expected = 0;
        while (terminated) {
            if (rebuildTree_) {
                // 如果有搜索操作在进行，那么就得等待搜索操作结束后才能进行插入，问题是，由于是RRT场景，插入和查询比较频繁，
                // 但是我们设定不平衡的树节点个数超过某个阈值后，才能进行重建，其实搜索和添加点花费的的时间都很短
                // 需要考虑以下的事
                // 1.重建和搜索并发，然后搜索可能访问空指针
                // 2.添加新点和重建并发，由于重建花的时间比较长，可能中间有多个树又需要重新构建，并且在构建的途中如果新的点是插入
                //   重构建的树下面，就会破坏原来的结构
                // 3.RRT中，添加点和搜索步骤是串行的，不需要考虑它们之间的多线程安全问题

                // 保存父节点以及树的节点信息
                KdTreeNode* old_root_node = (*rebuildTree_);
                rebuild_flag_ = true;
                parent = old_root_node->parent;

                PointVector().swap(rebuild_pcl_storage_);
                while (true) {
                    if (search_counter_.compare_exchange_strong(
                            expected, -1, std::memory_order_acq_rel, std::memory_order_acquire)) {
                        flatten(old_root_node, rebuild_pcl_storage_);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::nanoseconds(10));
                }

                // 允许搜索操作进行
                search_counter_.store(0);
                KdTreeNode* new_root_node{nullptr};
                if (!rebuild_pcl_storage_.empty()) {
                    // 重新构建树
                    buildTree(&new_root_node, rebuild_pcl_storage_);

                    // 阻塞进程，其实这时候有重构建标志，不一定需要这个保护队列
                    std::unique_lock<std::mutex> om{operation_mutex_};
                    om.lock();
                    while (!point_chache_.empty()) {
                        PointType point = point_chache_.front();
                        point_chache_.pop();
                        add_point(point, new_root_node);
                    }
                    om.unlock();
                }

                // 不允许搜索操作，准备把重构建的树替换
                while (true) {
                    if (search_counter_.compare_exchange_strong(
                            expected, -1, std::memory_order_acq_rel, std::memory_order_acquire))
                        break;

                    std::this_thread::sleep_for(std::chrono::nanoseconds(10));
                }

                //
                if (parent->right_child == old_root_node)
                    parent->right_child = new_root_node;
                else if (parent->left_child == old_root_node)
                    parent->left_child = new_root_node;
                else
                    std::cerr << "" << std::endl;

                if (new_root_node) new_root_node->parent = parent;

                while (true) {
                    if (search_counter_.compare_exchange_strong(
                            expected, 0, std::memory_order_acq_rel, std::memory_order_acquire)) {
                        rebuild_flag_.store(false);
                        rebuildTree_ = nullptr;
                    }

                    std::this_thread::sleep_for(std::chrono::nanoseconds(10));
                }
            }
        }
    }

    void buildTree(KdTreeNode** root, PointVector& points) {
        std::stack<MessageStorage> mss;

        std::optional<std::vector<MessageStorage>> msv =
            buildTree(root, 0, uint32_t(points.size() - 1), points);
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

    void deleteTree() {
        std::stack<KdTreeNode*> node_stack;
        if (root_) node_stack.push(root_);

        while (!node_stack.empty()) {
            KdTreeNode* node = node_stack.top();
            node_stack.pop();
            if (node->right_child) node_stack.push(node->right_child);
            if (node->left_child) node_stack.push(node->left_child);
            if (node) delete node;
        }

        root_ = nullptr;
    }

    std::optional<std::vector<KdTreeNode*>> search(
        KdTreeNode* root, int k_nearest, PointType point, ManualHeap& q, double max_dist) {
        std::vector<KdTreeNode*> child_nodes;

        if (!root) return std::nullopt;

        double cur_dist = calc_box_dist(root, point);
        double max_dist_sqr = max_dist * max_dist;
        if (cur_dist > max_dist_sqr) return std::nullopt;

        double dist = calc_dist(root->data, point);
        if (max_dist_sqr < dist || (dist < q.top().dist && q.size() < k_nearest)) {
            if (q.size() > k_nearest) q.pop();
            PointTypeCmp current_point(root->data, dist);
            q.push(current_point);
        }

        float dist_left_node = calc_box_dist(root->left_child, point);
        float dist_right_node = calc_box_dist(root->right_child, point);

        if (q.size() < k_nearest || dist_left_node < q.top().dist ||
            dist_right_node < q.top().dist) {
            if (q.size() < k_nearest) {
                child_nodes.emplace_back(root->left_child);
                child_nodes.emplace_back(root->right_child);
            } else {
                if (dist_left_node < q.top().dist) child_nodes.emplace_back(root->left_child);
                if (dist_right_node < q.top().dist) child_nodes.emplace_back(root->right_child);
            }
        }
        return child_nodes;
    }

    float calc_dist(PointType a, PointType b) { return (a - b).norm(); }

    float calc_box_dist(KdTreeNode* node, PointType point) {
        if (!node) return std::numeric_limits<float>::infinity();

        float min_dist{0.0};
        for (size_t i{0}; i < dim; ++i) {
            min_dist += point(i) < node->node_range(i, 0)
                            ? std::pow(point(i) - node->node_range(i, 0), 2)
                            : 0.0;
            min_dist += point(i) > node->node_range(i, 1)
                            ? std::pow(point(i) - node->node_range(i, 1), 2)
                            : 0.0;
        }

        return min_dist;
    }

    std::optional<KdTreeNode**> add_point(
        KdTreeNode** root, KdTreeNode* parent, PointType& point, bool allow_rebuild,
        uint8_t parent_axis) {
        KdTreeNode *node = *root, **div_node{nullptr};
        if (!node) {
            node = mp_.allocate();
            node->data = point;
            node->div_axis = (parent_axis + 1) % dim;

            for (int i{0}; i < dim; ++i) {
                node->node_range(i, 0) = point(i);
                node->node_range(i, 1) = point(i);
            }

            // 回溯更新父节点，检查是否需要重建树
            node->parent = parent;
            while (node) {
                if (node->parent) {
                    node->parent->tree_size += 1;
                    compareWithRangeAndUpdate(node->parent, node);
                }
                node = node->parent;
            }

            bool need_rebuild = allow_rebuild && criterionCheck(root);

            return std::nullopt;
        }

        bool is_left{false};

        for (uint8_t i{0}; i < dim; ++i) {
            if (node->data(i) < point(i)) is_left = true;
        }

        div_node = is_left ? &node->left_child : &node->right_child;

        return div_node;
    }

    void compareWithRangeAndUpdate(KdTreeNode* parent, KdTreeNode* child) {
        for (int i{0}; i < dim; ++i) {
            parent->node_range(i, 0) = std::min(parent->node_range(i, 0), child->node_range(i, 0));
            parent->node_range(i, 1) = std::max(parent->node_range(i, 1), child->node_range(i, 1));
        }
    }

    void flatten(KdTreeNode* root, PointVector& storage) {
        storage.clear();
        if (!root) return;
        std::stack<KdTreeNode*> ns;
        ns.emplace(root);
        storage.reserve(root->tree_size);
        storage.emplace_back(root->data);

        KdTreeNode *parent{nullptr}, *rc{nullptr}, *lc{nullptr};
        if (!ns.empty()) {
            parent = ns.top();
            rc = parent->right_child;
            lc = parent->left_child;
            ns.pop();
            if (lc) {
                storage.emplace_back(lc->data);
                ns.emplace(lc);
            }
            if (rc) {
                storage.emplace_back(rc);
                ns.emplace(rc);
            }
        }
    }

    bool criterionCheck(KdTreeNode* root) {
        if (root->TreeSize <= minimal_unbalanced_tree_size) {
            return false;
        }
        float balance_evaluation = 0.0f;
        KdTreeNode* son_ptr = root->left_child;
        if (son_ptr == nullptr) son_ptr = root->right_son_ptr;
        balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        if (balance_evaluation > balance_criterion_param ||
            balance_evaluation < 1 - balance_criterion_param) {
            return true;
        }
        return false;
    }

    void freeMemory(KdTreeNode* root) {
        if (!root) return;

        std::stack<KdTreeNode*> ns;
        ns.emplace(root);

        KdTreeNode *parent{nullptr}, *rc{nullptr}, *lc{nullptr};
        if (!ns.empty()) {
            parent = ns.top();
            ns.pop();
            rc = parent->right_child;
            lc = parent->left_child;
            if (rc) ns.emplace(rc);
            if (lc) ns.emplace(lc);
        }

        parent = root->parent;
        if (parent && parent->left_child == root) {
            parent->left_child = nullptr;
            return;
        }

        if (parent && parent->right_child == root) {
            parent->right_child = nullptr;
            return;
        }
    }

    void rebuild(KdTreeNode** root) {
        KdTreeNode* parent{nullptr};
        if ((*root)->tree_size >= multi_thread_rebuild_point_num) {
        } else {
            parent = (*root)->parent;
            int size_rec = (*root)->tree_size;
            pcl_storage_.clear();
            flatten(root, pcl_storage_);
            build(root, 0, pcl_storage_.size() - 1, pcl_storage_);
            if (*root) (*root)->parent = parent;
        }
    }

    void add_point(PointType point, KdTreeNode* root) {
        std::stack<KdTreeNode**> ns;

        if (!root) return;

        std::optional<KdTreeNode**> result =
            add_point(&root, root->parent, point, true, root_->div_axis);
        if (result.has_value() && result.value() != nullptr) {
            ns.push(result.value());
        }

        KdTreeNode** n{nullptr};
        KdTreeNode* parent = root;
        uint8_t parent_axis = root->div_axis;

        while (!ns.empty()) {
            n = ns.top();
            ns.pop();
            result = add_point(n, parent, point, true, parent_axis);
            if (result.has_value() && result.value() != nullptr) {
                ns.push(result.value());
                parent_axis = (*n)->div_axis;
                parent = (*n);
            } else
                break;
        }
    }

public:
    NearestNeighbors(float balance_param) : balance_criterion_param_(balance_param) {
        cmp_funv_.resize(dim);
        for (uint8_t i{0}; i < dim; ++i)
            cmp_funv_[i] = [i](PointType a, PointType b) { return a(i) < b(i); };
    }

    ~NearestNeighbors() { freeMemory(root_); }

    void build(PointVector& points) {
        if (root_) deleteTree();

        if (points.empty()) return;

        buildTree(&root_, points);
    }

    void searchNearest(
        PointType point, int k_nearest, PointVector& nearest_points, std::vector<float>& point_dist,
        double max_dist = std::numeric_limits<double>::infinity()) {
        ManualHeap q(2 * k_nearest);
        q.clear();

        std::vector<float>().swap(point_dist);

        std::stack<KdTreeNode*> node_stack;
        if (root_) {
            if (root_ == rebuildTree_) {
                std::optional<std::vector<KdTreeNode*>> result =
                    search(root_, k_nearest, point, q, max_dist);
                if (result.has_value()) {
                    for (auto element : result.value()) node_stack.push(element);
                }
            }
        }

        std::queue<KdTreeNode*> rebuild_trees_;
        while (!node_stack.empty()) {
            KdTreeNode* node = node_stack.top();
            node_stack.pop();

            std::optional<std::vector<KdTreeNode*>> result =
                search(node, k_nearest, point, q, max_dist);

            if (result.has_value()) {
                for (auto element : result.value()) node_stack.push(element);
            }
        }

        int k_found = std::min(k_nearest, int(q.size()));

        PointVector().swap(nearest_points);
        std::vector<float>().swap(point_dist);
        for (int i{0}; i < k_found; ++i) {
            nearest_points.insert(nearest_points.begin(), q.top().point);
            point_dist.insert(point_dist.begin(), q.top().dist);
            q.pop();
        }
    }

    void add(PointType point) {
        add_point(point, root_);
    }
};
}  // namespace fast_motion_planning

#endif
