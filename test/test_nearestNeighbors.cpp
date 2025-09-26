// cpp
#include <algorithm>
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
    fmp::NearestNeighbor<Data, 6, 20000> nn;
    fmp::RandomNumberGenerator rng;

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Data> data_set;

    Data data;
    for (size_t i{0}; i < 20000; ++i) {
        for (size_t j{0}; j < 6; ++j) {
            data.point(j) = rng.uniform(11113.14, 22023.14);
        }
        data_set.emplace_back(data);
    }

    nn.build(data_set);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树构建耗时:" << std::chrono::duration<double, std::milli>(end - start).count()
              << "ms\n";

    data.point = Eigen::Vector<double, 6>{2000, 1000, 1250, 1000, 1000, 1000};

    start = std::chrono::high_resolution_clock::now();
    auto point = data.point;
    std::sort(data_set.begin(), data_set.end(), [point](auto& a, auto& b) {
        return (a.point - point).transpose() * (a.point - point) <
               (b.point - point).transpose() * (b.point - point);
    });
    // std::sort(data_set.begin(), data_set.end(), [point](auto a, auto b) {
    //     return (a.point - point).norm() < (b.point - point).norm();
    // });
    end = std::chrono::high_resolution_clock::now();
    std::cout << "暴力排序查询最邻近点:"
              << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";

    start = std::chrono::high_resolution_clock::now();
    nn.searchNearest(point, data);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "KD树查询最邻近点:"
              << std::chrono::duration<double, std::milli>(end - start).count() << "ms\n";

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "暴力排序最邻近点" << std::endl;
    std::cout << data_set[0].point.format(CleanFmt) << std::endl;
    std::cout << "Kd-tree搜索最邻近点" << std::endl;
    std::cout << data.point.format(CleanFmt) << std::endl;
    // for (int i{0}; i < 70000; ++i) {
    //     for (size_t j{0}; j < 6; ++j) {
    //         point(j) = rng.uniform(2113.14, 1223.14);
    //     }
    //     nn.add(point);
    // }
    // auto cur_time = std::chrono::high_resolution_clock::now();
    // std::cout << "KD树插入点耗时:"
    //           << std::chrono::duration<double, std::milli>(cur_time - end).count() << "ms\n";

    // point(0) = 2000.0f;
    // point(1) = 1000.0f;
    // point(2) = -200.0f;
    // point(3) = 780.0f;
    // point(4) = 50.0f;
    // point(5) = -1000.0f;

    // points.clear();

    // std::vector<float> dist;
    // start = std::chrono::high_resolution_clock::now();
    // nn.searchNearest(point, 1, points, dist);

    // for (auto nearest_point : points) {
    //     Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //     std::cout << std::endl;
    //     std::cout << nearest_point.format(CleanFmt) << std::endl;
    // }
    // end = std::chrono::high_resolution_clock::now();
    // std::cout << "查询耗时:" << std::chrono::duration<double, std::milli>(end - start).count()
    //           << "ms\n";
    // return 0;
}
