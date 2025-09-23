#include <chrono>
#include <cstddef>
#include <fast_motion_planning/NearestNeighbors.hpp>
#include <fast_motion_planning/sampler.hpp>
#include <ratio>

int main() {
    namespace fmp = fast_motion_planning;
    fmp::NearestNeighbors<6> nn(0.65);
    fmp::RandomNumberGenerator rng;
    fmp::NearestNeighbors<6>::PointVector points;
    fmp::NearestNeighbors<6>::PointType point;

    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i{0}; i < 2000; ++i) {
        for (size_t j{0}; j < 6; ++j) {
            point(j) = rng.uniform(1113.14, -3223.14);
        }
        points.emplace_back(point);
    }

    nn.build(points);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树构建耗时:" << std::chrono::duration<double, std::milli>(end - start).count()
              << "ms\n";
    for (int i{0}; i < 90000; ++i) {
        for (size_t j{0}; j < 6; ++j) {
            point(j) = rng.uniform(2113.14, 1223.14);
        }
        nn.add_point(point);
    }
    auto cur_time = std::chrono::high_resolution_clock::now();
    std::cout << "KD树插入点耗时:"
              << std::chrono::duration<double, std::milli>(cur_time - end).count() << "ms\n";
    return 0;
}
