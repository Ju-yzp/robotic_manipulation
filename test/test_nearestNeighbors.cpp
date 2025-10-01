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
        Eigen::Vector<double, 2> point;
    };
    fmp::NearestNeighbor<Data, 2> nn(100);
    fmp::RandomNumberGenerator rng;

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Data> data_set;

    Data data;
    Data data1{Eigen::Vector<double, 2>{100, -100}};
    Data data2{Eigen::Vector<double, 2>{100, 100}};
    Data data3{Eigen::Vector<double, 2>{100, 200}};
    data_set.emplace_back(data1);
    data_set.emplace_back(data2);
    data_set.emplace_back(data3);
    nn.build(data_set);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树构建耗时:" << std::chrono::duration<double, std::milli>(end - start).count()
              << "ms\n";

    // data.point = Eigen::Vector<double, 6>{12000, 41000, 6250, 8000, 11000, 10000};

    // nn.insert(data);
    start = std::chrono::high_resolution_clock::now();
    for (size_t i{0}; i < 30; ++i) {
        for (size_t j{0}; j < 2; ++j) {
            data.point(j) = rng.uniform(110, 20000);
        }
        nn.add_point(data);
        // nn.iter();
        //  auto point = data.point;
        //  nn.search(point, data);
    }
    end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树查询最邻近点:"
              << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";
    return 0;
}
