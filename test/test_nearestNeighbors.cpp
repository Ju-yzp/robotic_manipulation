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
    fmp::NearestNeighbor<Data, 2, 100000> nn;
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
    for (size_t i{0}; i < 10000; ++i) {
        for (size_t j{0}; j < 2; ++j) {
            data.point(j) = rng.uniform(110, 20000);
        }
        nn.insert(data);
        // data_set.emplace_back(data);
        //  start = std::chrono::high_resolution_clock::now();
        auto point = data.point;
        // std::sort(data_set.begin(), data_set.end(), [point](auto a, auto b) {
        //     return (a.point - point).norm() < (b.point - point).norm();
        // });
        // end = std::chrono::high_resolution_clock::now();
        // std::cout << "暴力排序查询最邻近点:"
        //           << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";

        // start = std::chrono::high_resolution_clock::now();
        nn.searchNearest(point, data);
        // end = std::chrono::high_resolution_clock::now();
        // std::cout << "KD树查询最邻近点:"
        //           << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";

        // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        // std::cout << "暴力排序最邻近点" << std::endl;
        // std::cout << data_set[0].point.format(CleanFmt) << std::endl;
        // std::cout << "Kd-tree搜索最邻近点" << std::endl;
        // std::cout << data.point.format(CleanFmt) << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树查询最邻近点:"
              << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";
    return 0;
}
