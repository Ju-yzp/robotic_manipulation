#ifndef FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_
#define FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_

// cpp
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <stack>

// eigen
#include <Eigen/Eigen>

namespace fast_motion_planning {

// 单线程版本
template <size_t dim>
class NearestNeighbors {  // TODO：下面全部使用的是欧氏距离
public:
    using PointType = Eigen::Vector<double, dim>;
    using PointVector = std::vector<PointType>;

private:
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
    };

    using CmpFunc = std::function<bool(PointType, PointType)>;

    // 比较函数
    std::vector<CmpFunc> cmp_funv_;

    // 平衡系数
    float balance_criterion_param_{0.7};

    // 平衡中间数
    float alpha_bal_tmp_{0.5};

    KdTreeNode* root_{nullptr};

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

    // 堆
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
            heap_size--;
            moveDown(0);
            return;
        }

        void push(PointTypeCmp point) {
            if (heap_size >= capacity) return;
            heap[heap_size] = point;
            floatUp(heap_size);
            --heap_size;
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

        if (!(*root)) *root = new KdTreeNode();
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
            node->left_child = new KdTreeNode();
            ms.node = node->left_child;
            ms.start = start;
            ms.end = mid - 1;
            ms.node->parent = node;
            msv.emplace_back(ms);
        }
        if (mid + 1 <= end) {
            node->right_child = new KdTreeNode();
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

    void deleteTree() {  // 后面会使用固定大小内存池分配内存，减少系统调用的使用
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

        float dist_left_node = calc_box_dist(root->left_child, point);
        float dist_right_node = calc_box_dist(root->right_child, point);

        if (q.size() < k_nearest || dist_left_node < q.top().dist ||
            dist_right_node < q.top().dist) {
            if (dist_left_node < q.top().dist) child_nodes.emplace_back(root->left_child);
            if (dist_right_node < q.top().dist) child_nodes.emplace_back(root->right_child);
        }
        return child_nodes;
    }

    float calc_dist(PointType a, PointType b) { return (a - b).norm(); }

    float calc_box_dist(KdTreeNode* node, PointType point) {
        if (!node) return std::numeric_limits<float>::infinity();

        float min_dist{0.0};
        for (size_t i{0}; i < dim; ++i) {
            min_dist += point(i) < node->node_range(i)(0)
                            ? std::pow(point(i) - node->node_range(i)(0), 2)
                            : 0.0;
            min_dist += point(i) > node->node_range(i)(1)
                            ? std::pow(point(i) - node->node_range(i)(1), 2)
                            : 0.0;
        }

        return min_dist;
    }

    std::optional<KdTreeNode**> add_point(
        KdTreeNode** root, KdTreeNode* parent, PointType& point, bool allow_rebuild,
        uint8_t parent_axis) {
        KdTreeNode *node = *root, **div_node{nullptr};
        if (!node) {
            node = new KdTreeNode();
            std::cout << node << std::endl;
            node->data = point;
            node->div_axis = (parent_axis + 1) % dim;

            for (int i{0}; i < dim; ++i) {
                node->node_range(i, 0) = point(i);
                node->node_range(i, 1) = point(i);
            }

            node->parent = parent;
            while (node) {
                if (node->parent) {
                    node->parent->tree_size += 1;
                    compareWithRangeAndUpdate(node->parent, node);
                }
                node = node->parent;
            }
            return std::nullopt;
        }

        bool is_left{false};

        for (uint8_t i{0}; i < dim; ++i) {
            if (node->data(i) < point(i)) is_left = true;
        }

        div_node = is_left ? &node->left_child : &node->right_child;

        // 检查是否需要重建子树
        return div_node;
    }

    void compareWithRangeAndUpdate(KdTreeNode* parent, KdTreeNode* child) {
        for (int i{0}; i < dim; ++i) {
            parent->node_range(i, 0) = std::min(parent->node_range(i, 0), child->node_range(i, 0));
            parent->node_range(i, 1) = std::max(parent->node_range(i, 1), child->node_range(i, 1));
        }
    }

public:
    NearestNeighbors(float balance_param) : balance_criterion_param_(balance_param) {
        cmp_funv_.resize(dim);
        for (uint8_t i{0}; i < dim; ++i)
            cmp_funv_[i] = [i](PointType a, PointType b) { return a(i) < b(i); };
    }

    ~NearestNeighbors() { deleteTree(); }

    void build(PointVector& points) {
        if (root_) deleteTree();

        if (points.empty()) return;

        buildTree(&root_, points);
    }

    void searchNearest(
        PointType point, int k_nearest, PointVector& nearest_points, std::vector<float>& point_dist,
        double max_dist = std::numeric_limits<double>::infinity()) {
        ManualHeap q(2 * k_nearest);

        std::stack<KdTreeNode*> node_stack;
        if (root_) node_stack.push(root_);

        while (!node_stack.empty()) {
            KdTreeNode* node = node_stack.top();
            node_stack.pop();

            std::optional<std::vector<KdTreeNode*>> result =
                search(node, k_nearest, point, q, max_dist);

            if (result.has_value()) {
                for (auto element : result) node_stack.push(element);
            }
        }

        int k_found = std::min(k_nearest, int(q.size()));
        PointVector().swap(nearest_points);
        for (int i{0}; i < k_found; ++i) {
        }
    }

    void add_point(PointType point) {
        std::stack<KdTreeNode**> ns;

        if (!root_) return;

        std::optional<KdTreeNode**> result =
            add_point(&root_, root_->parent, point, true, root_->div_axis);
        if (result.has_value() && result.value() != nullptr) {
            ns.push(result.value());
        }

        KdTreeNode** n{nullptr};
        KdTreeNode* parent = root_;
        uint8_t parent_axis = root_->div_axis;

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
};
}  // namespace fast_motion_planning

#endif
