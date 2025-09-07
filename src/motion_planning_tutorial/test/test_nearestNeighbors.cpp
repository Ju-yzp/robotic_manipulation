// motion_planning_tutorial
#include <motion_planning_tutorial/NearestNeighbors.hpp>
#include <motion_planning_tutorial/state.hpp>

#include <iostream>

namespace mpt = motion_planning_tutorial;
struct TestNode {
    mpt::State state;
};

int main() {
    mpt::NearestNeighbors<TestNode> nn;
    TestNode n1, n2, n3, n4;
    n1.state.positions = Eigen::VectorXd::Zero(3);
    n2.state.positions = Eigen::VectorXd::Ones(3);
    n3.state.positions = 2 * Eigen::VectorXd::Ones(3);
    nn.add(&n1);
    nn.add(&n2);
    nn.add(&n3);
    Eigen::Vector3d pos;
    pos << 0.4, 2.6, 7.8;
    n4.state.positions = pos;
    auto nearest = nn.searchNearestNeighbor(&n4);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << nearest->state.positions.format(CleanFmt) << std::endl;

    auto nearests = nn.searchNearestNeighborsWithRadius(10.0, &n4);
    for (const auto& n : nearests) {
        std::cout << n->state.positions.format(CleanFmt) << std::endl;
    }
}
