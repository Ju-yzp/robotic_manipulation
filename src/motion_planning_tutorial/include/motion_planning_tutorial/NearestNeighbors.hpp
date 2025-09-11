/*
Description: NearestNeighbors: a class for describing nearest neighbors search
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_NEARESTNEIGHBORS_HPP_
#define MOTION_PLANNING_TOTURIAL_NEARESTNEIGHBORS_HPP_

// cpp
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <vector>

namespace motion_planning_tutorial {

template <typename T>
class NearestNeighbors {
public:
    void add(T* element) { data_.emplace_back(element); }

    T* searchNearestNeighbor(T* element) {
        double distance = std::numeric_limits<double>::infinity();
        T* nearestNeighbor = nullptr;
        for (const auto& d : data_) {
            assert(d->state.positions.rows() == element->state.positions.rows());
            double dist = (d->state.positions - element->state.positions).norm();
            if (dist < distance) {
                distance = dist;
                nearestNeighbor = d;
            }
        }
        return nearestNeighbor;
    }

    std::size_t size() { return data_.size(); }

    std::vector<T*> searchNearestNeighborsWithRadius(const double radius, T* element) {
        std::vector<T*> elements;
        std::for_each(data_.begin(), data_.end(), [&](auto a) {
            double dist = (a->state.positions - element->state.positions).norm();
            if (dist < radius * radius) elements.emplace_back(a);
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

    void relaseMemoryResource() {
        for (auto& ptr : data_) {
            if (ptr) {
                delete ptr;
                ptr = nullptr;
            }
        }

        data_.clear();
    }

private:
    // 数据集合
    std::vector<T*> data_;
};

}  // namespace motion_planning_tutorial
#endif
