#include <cstddef>
#include <fast_motion_planning/NearestNeighbors.hpp>
#include <fast_motion_planning/sampler.hpp>

int main() {
    namespace fmp = fast_motion_planning;
    fmp::NearestNeighbors<6> nn(0.65);
    fmp::RandomNumberGenerator rng;

    fmp::NearestNeighbors<6>::PointVector points;
    fmp::NearestNeighbors<6>::PointType point;
    for (size_t i{0}; i < 100000; ++i) {
        for (size_t j{0}; j < 6; ++j) {
            point(j) = rng.uniform(113.14, -223.14);
        }
        points.emplace_back(point);
    }

    nn.build(points);

    for (size_t j{0}; j < 6; ++j) {
        point(j) = rng.uniform(2113.14, 1223.14);
    }

    nn.add_point(point);
    return 0;
}
