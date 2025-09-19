#ifndef FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_
#define FAST_MOTION_PLANNING_NEAREST_NEIGHBORS_HPP_

#include <algorithm>
#include <cstddef>
#include <vector>

namespace fast_motion_planning {

template <typename T>
class NearestNeighbors {  // TODO：后面会使用kd树进行
public:
    std::size_t size() { return data_.size(); }

    void reset() { data_.clear(); }

    void insert(T* element) { data_.push_back(element); }

    std::vector<T*> searchNearestNeighborsByRadius(const double radius, const T* element) {
        std::vector<T*> elements;
        std::for_each(data_.begin(), data_.end(), [&](T* data) {
            if ((data->state - element->state).norm() < radius) elements.emplace_back(data);
        });

        return elements;
    }

    std::vector<T*> serachKNearestNeighbors(const int k, T* element) {
        std::vector<T*> elements;
        elements.resize(k + 1);
        std::sort(data_.begin(), data_.end(), [&](auto a, auto b) {
            double dist_a = (a->state.positions - element->state.positions).norm();
            double dist_b = (b->state.positions - element->state.positions).norm();
            return dist_a < dist_b;
        });

        size_t count = std::min(data_.size(), static_cast<size_t>(k));
        for (size_t i = 0; i < count; ++i) {
            elements[i] = data_[i];
        }
    }

private:
    std::vector<T*> data_;  // 数据
};
}  // namespace fast_motion_planning

#endif
