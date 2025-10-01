#ifndef FAST_MOTION_PLANNING_SAMPLER_HPP_
#define FAST_MOTION_PLANNING_SAMPLER_HPP_

// fast_motion_planning
#include <fast_motion_planning/RobotParams.hpp>
#include <fast_motion_planning/types.hpp>

// cpp
#include <cstddef>
#include <iostream>
#include <random>

// eigen
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/Eigen>

namespace fast_motion_planning {

class RandomNumberGenerator {  // 随机数字生成器
public:
    double uniform01() { return uniDist_(generator_); }

    double uniform(const double upper, const double lower) {
        return lower + (upper - lower) * uniform01();
    }

    int uniformInt(const int upper, const int lower) {
        auto r = (int)floor(uniform((double)lower, (double)(upper) + 1.0));
        return (r > upper) ? upper : r;
    }

    double gaussian01() { return normalDist_(generator_); }

    double gaussian(double mean, double stddev);

    Eigen::Quaterniond quaternion();

    Eigen::Vector3d eulerRPY();

private:
    std::mt19937 generator_;

    // 正态分布
    std::normal_distribution<> normalDist_{0, 1};

    // 均匀分布
    std::uniform_real_distribution<> uniDist_{0, 1};
};

class Sampler {
public:
    REGISTER_SMART_POINTER(Sampler)

    Sampler(const RobotParams::SharedPtr& robot_params);

    Sampler() = delete;

    State sample(size_t start, size_t expand);

    State sample();

    bool insert(Eigen::Vector3d state);

private:
    // 三维空间，对应UR5E的前三个轴
    class SpaceManager {
    public:
        struct SpaceNode {
            int32_t num{0};          // 空间内点数量
            Eigen::Vector3d center;  // 中心
            Eigen::Vector3d extent;  // 中心在每个轴上到边界的距离
        };

        SpaceManager() {}

        void initilize(
            double min_x, double max_x, double min_y, double max_y, double min_z, double max_z,
            double resolution) {
            resolution_ = resolution;
            SpaceNode root;
            root.center(0) = (max_x + min_x) / 2.0;
            root.center(1) = (max_y + min_y) / 2.0;
            root.center(2) = (max_z + min_z) / 2.0;
            root.extent(0) = (max_x - min_x) / 2.0;
            root.extent(1) = (max_y - min_y) / 2.0;
            root.extent(2) = (max_z - min_z) / 2.0;
            activate_spaces_.emplace_back(root);
            std::vector<SpaceNode> temp_space_nodes;
            while (!checkResolution(activate_spaces_)) {
                temp_space_nodes.reserve(activate_spaces_.size() * 8);
                for (auto& sn : activate_spaces_) {
                    std::vector<SpaceNode> sns;
                    sns.reserve(8);
                    sns = divideSpace(sn);
                    temp_space_nodes.insert(temp_space_nodes.end(), sns.begin(), sns.end());
                }
                activate_spaces_ = temp_space_nodes;
                temp_space_nodes.clear();
            }

            // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
            // for(auto an:activate_spaces_){
            //     std::cout<<an.center.format(CleanFmt)<<std::endl;
            //     std::cout<<an.extent.format(CleanFmt)<<std::endl;
            //     std::cout<<std::endl;
            // }
        }

        std::vector<SpaceNode> get_activate_spaces() const { return activate_spaces_; }

        bool insert(Eigen::Vector3d point) {
            for (auto it = activate_spaces_.begin(); it != activate_spaces_.end();) {
                auto& asn = *it;

                if (point(0) > asn.center(0) + asn.extent(0) ||
                    point(0) < asn.center(0) - asn.extent(0) ||
                    point(1) > asn.center(1) + asn.extent(1) ||
                    point(1) < asn.center(1) - asn.extent(1) ||
                    point(2) > asn.center(2) + asn.extent(2) ||
                    point(2) < asn.center(2) - asn.extent(2)) {
                    ++it;
                } else {
                    ++asn.num;
                    if (asn.num > threshold_) {
                        std::cout << "!" << std::endl;
                        activate_spaces_.erase(it);
                        return false;
                    }

                    break;
                }
            }

            return true;
        }

    private:
        // 活跃空间
        std::vector<SpaceNode> activate_spaces_;

        // 分辨率
        double resolution_;

        // 阈值
        int32_t threshold_{1000};

        // 检查分辨率是否满足要求
        bool checkResolution(std::vector<SpaceNode>& space_nodes) {
            if (space_nodes.empty()) return false;

            auto sne = space_nodes[0].extent;
            if (sne(0) > resolution_ || sne(1) > resolution_ || sne(2) > resolution_) return false;

            return true;
        }

        // 分裂函数
        std::vector<SpaceNode> divideSpace(const SpaceNode& root) {
            std::vector<SpaceNode> child_space;
            child_space.resize(8);

            for (int i{0}; i < 8; ++i) {
                child_space[i].extent = root.extent / 2.0;
                child_space[i].center(0) =
                    root.center(0) + ((i & 0b100) ? root.extent(0) / 2.0 : -root.extent(0) / 2.0);
                child_space[i].center(1) =
                    root.center(1) + ((i & 0b010) ? root.extent(1) / 2.0 : -root.extent(1) / 2.0);
                child_space[i].center(2) =
                    root.center(2) + ((i & 0b001) ? root.extent(2) / 2.0 : -root.extent(2) / 2.0);
            }

            return child_space;
        }
    };

    // 使用两个维度进行采样时，会记录上个维度的值
    double last_state_;

    // 随机数字生成器
    RandomNumberGenerator rng_;

    // 约束矩阵(关节位置限制)
    Eigen::MatrixXd constraints_;

    // 维数
    size_t dim_;

    // 空间管理者
    SpaceManager space_manager_;

    // 使用空间管理者标志
    bool is_use_sn_{false};
};
}  // namespace fast_motion_planning

#endif
