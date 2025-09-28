#ifndef MOTION_PLANNING_TUTORIAL_NON_UNIFORM_BASPLINE_HPP_
#define MOTION_PLANNING_TUTORIAL_NON_UNIFORM_BASPLINE_HPP_

// eigen
#include <Eigen/Eigen>

// motion_planning_tutorial
#include <fast_motion_planning/RobotParams.hpp>

namespace fast_motion_planning {
class NonUniformBspline {
public:
    NonUniformBspline() = delete;

    NonUniformBspline(
        const Eigen::MatrixXd& control_points, const int order, const double& interval,
        const RobotParams::SharedPtr robot_params);

    ~NonUniformBspline() {}

    // 求导后的非均匀B样条
    NonUniformBspline getDerivative(const RobotParams::SharedPtr& robot_params);

    static void parameterizeToBspline(
        const double& ts, const std::vector<Eigen::Vector3d>& point_set,
        const std::vector<Eigen::Vector3d>& start_end_derivative, Eigen::MatrixXd& ctrl_pts);

    void setUniformBspline(const Eigen::MatrixXd& points, const int& order, const double& interval);

    Eigen::VectorXd evaluateDeBoor(const double& u);

    Eigen::VectorXd evaluateDeBoorT(const double& t);

    void set_knot(const Eigen::VectorXd& knot) { u_ = knot; }

    void getTimeSpan(double& um, double& um_p);

    Eigen::VectorXd get_knot() { return u_; }

    Eigen::MatrixXd get_control_points() { return control_points_; }

    double get_interval() { return interval_; }

    void set_has_acceleration_limit(bool has_acceleration_limit) {
        has_acceleration_limit_ = has_acceleration_limit;
    }

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

    Eigen::VectorXd velocity_limit_;

    Eigen::VectorXd acceleration_limit_;
};
}  // namespace fast_motion_planning
#endif
