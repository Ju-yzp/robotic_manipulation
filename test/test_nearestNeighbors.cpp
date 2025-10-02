// cpp
#include <chrono>
#include <cstddef>
#include <ratio>

// fast_motion_planning
#include <fast_motion_planning/NearestNeighbor.hpp>
#include <fast_motion_planning/sampler.hpp>

int main() {
    namespace fmp = fast_motion_planning;
    struct Data {
        Eigen::Vector<double, 6> point;
    };
    fmp::NearestNeighbor<Data, 6> nn(510000);
    fmp::RandomNumberGenerator rng;

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Data> data_set;

    Data data;
    Data data1{Eigen::Vector<double, 6>{100, -100, 0, 0, 0, 0}};
    data_set.emplace_back(data1);
    nn.build(data_set);
    auto end = std::chrono::high_resolution_clock::now();
    auto point = Eigen::Vector<double, 6>{200.0, 400.0, 1000.0, 2000.0, 100.0, 200.0};
    std::cout << "KD树构建耗时:" << std::chrono::duration<double, std::milli>(end - start).count()
              << "ms\n";

    start = std::chrono::high_resolution_clock::now();
    for (size_t i{0}; i < 401000; ++i) {
        for (size_t j{0}; j < 6; ++j) {
            data.point(j) = rng.uniform(110, 20000);
        }
        nn.add_point(data);

        nn.search(point, data);
    }
    end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树查询最邻近点:"
              << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";
    return 0;
}
