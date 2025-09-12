#ifndef MOTION_PLANNING_TUTORIAL_NON_UNIFORM_BASPLINE_HPP_
#define MOTION_PLANNING_TUTORIAL_NON_UNIFORM_BASPLINE_HPP_

// eigen
#include <Eigen/Eigen>

// motion_planning_tutorial
#include <motion_planning_tutorial/robot_description.hpp>

// cpp
#include <unordered_map>

namespace motion_planning_tutorial {
class NonUniformBspline {
public:
    NonUniformBspline() = delete;

    NonUniformBspline(
        const Eigen::MatrixXd& control_points, const int order, const double& interval,
        const RobotDescription::SharedPtr& robot_description,
        const std::unordered_map<size_t, std::string>& id_map);

    ~NonUniformBspline() {}

    // 求导后的非均匀B样条
    NonUniformBspline getDerivative();

    Eigen::VectorXd evaluateDeBoor(const double& u);

    Eigen::VectorXd evaluateDeBoorT(const double& t);

    // 设置/获取B样条信息 TODO:应该添加判断维数
    void set_knot(const Eigen::VectorXd& knot) { u_ = knot; }

    void getTimeSpan(double& um, double& um_p);

    Eigen::VectorXd get_knot() { return u_; }

    Eigen::MatrixXd get_control_points() { return control_points_; }

    double get_interval() { return interval_; }

    void set_has_acceleration_limit(bool has_acceleration_limit) {
        has_acceleration_limit_ = has_acceleration_limit;
    }

    // 检查可达性
    bool checkFeasiblity();

    bool reallocateTime();

    double getTimeSum();

private:
    Eigen::MatrixXd getDerivativeControlPoints();

    // 控制点
    Eigen::MatrixXd control_points_;

    // 次数， n + 1个控制点， m = n + p + 1
    int p_, n_, m_;

    // 节点向量
    Eigen::VectorXd u_;

    double interval_;

    double artio_limit_{1.04};
    bool has_acceleration_limit_{false};

    // 机器人描述
    RobotDescription::SharedPtr robot_description_;

    //
    std::unordered_map<size_t, std::string> id_map_;

    Eigen::VectorXd velocity_limit_;

    Eigen::VectorXd acceleration_limit_;
};
}  // namespace motion_planning_tutorial
#endif
