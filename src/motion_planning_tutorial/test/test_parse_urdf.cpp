#include <visualization_module/trajectory_visualization.hpp>

int main() {
    std::string model_description = "/home/up/robot_description/urdf/robot.urdf";

    TrajectoryVisualization tv(model_description);
    return 0;
}
