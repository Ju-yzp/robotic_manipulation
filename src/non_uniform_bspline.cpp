// cpp
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>

// motion_palnning_tutorial
#include <fast_motion_planning/non_uniform_bspline.hpp>
#include "fast_motion_planning/RobotParams.hpp"

namespace fast_motion_planning {

NonUniformBspline::NonUniformBspline(
    const Eigen::MatrixXd& control_points, const int order, const double& interval,
    const RobotParams::SharedPtr robot_params) {
    control_points_ = control_points;
    p_ = order;
    interval_ = interval;

    n_ = control_points.rows() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i) {
        if (i <= p_) {
            u_(i) = double(-p_ + i) * interval_;
        } else if (i > p_ && i <= m_ - p_) {
            u_(i) = u_(i - 1) + interval_;
        } else if (i > m_ - p_) {
            u_(i) = u_(i - 1) + interval_;
        }
    }

    const int dof = robot_params->get_dof();
    velocity_limit_ = Eigen::VectorXd::Zero(dof);
    acceleration_limit_ = Eigen::VectorXd::Zero(dof);

    for (size_t i{0}; i < dof; ++i) {
        const auto joint_limit = robot_params->get_jointlimit(i);
        if (joint_limit.has_value()) {
            velocity_limit_(i) = joint_limit->velocity;
            acceleration_limit_(i) = joint_limit->acceleration;
        }
    }
}

bool NonUniformBspline::checkFeasiblity() {
    Eigen::MatrixXd p = control_points_;
    const int dim = control_points_.cols();

    //  检查速度可行性
    for (int i{0}; i < p.rows() - 1; ++i) {
        Eigen::VectorXd vec = p_ * (p.row(i + 1) - p.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
        for (int j{0}; j < dim; ++j)
            if (fabs(vec(j)) > velocity_limit_(j)) return false;
    }

    // 检查加速度可行性
    if (has_acceleration_limit_) {
        double max_vel = std::numeric_limits<double>::lowest();
        for (int i{0}; i < p.rows() - 2; ++i) {
            Eigen::VectorXd acc = ((p.row(i + 2) - p.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                                   (p.row(i + 1) - p.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                                  (u_(i + p_ + 1) - u_(i + 2));
            for (int j{0}; j < dim; ++j)
                if (fabs(acc(j)) > acceleration_limit_(j)) return false;
        }
    }

    return true;
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {
    double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

    int k = p_;
    while (true) {
        if (u_(k + 1) >= ub) break;
        ++k;
    }

    std::vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i) {
        d.push_back(control_points_.row(k - p_ + i));
    }

    for (int r = 1; r <= p_; ++r) {
        for (int i = p_; i >= r; --i) {
            double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
            d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
        }
    }

    return d[p_];
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
    return evaluateDeBoor(t + u_(p_));
}

bool NonUniformBspline::reallocateTime() {
    bool feasible{true};

    Eigen::MatrixXd p = control_points_;
    int dim = control_points_.cols();

    double max_vel_artio = std::numeric_limits<double>::lowest();
    // 检查速度可行性
    for (int i{0}; i < p.rows() - 1; ++i) {
        Eigen::VectorXd vec = p_ * (p.row(i + 1) - p.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
        bool flag{true};
        for (int j{0}; j < dim; ++j) {
            if (fabs(vec(j)) > velocity_limit_(j) + 1e-4) {
                feasible = false;
                flag = false;
                break;
            }
        }
        if (flag) continue;
        for (int j{0}; j < dim; ++j)
            max_vel_artio = std::max(max_vel_artio, fabs(vec(j) / velocity_limit_(j)));

        max_vel_artio = max_vel_artio < artio_limit_ ? max_vel_artio : artio_limit_ + 1e-4;

        // 重新分配时间，也就是重新给节点赋值
        double time_ori = u_(i + p_ + 1) - u_(i + 1);
        double time_new = time_ori * max_vel_artio;
        double delta_t = time_new - time_ori;
        double time_inc = delta_t / double(p_);

        for (int j{i + 2}; j < i + p_ + 2; ++j) u_(j) += double(j - i - 1) * time_inc;

        for (int j{i + p_ + 2}; j < u_.rows(); ++j) u_(j) += delta_t;
    }

    // 检查加速度可行性
    if (has_acceleration_limit_) {
        double max_acc_artio = std::numeric_limits<double>::lowest();
        for (int i{0}; i < p.rows() - 2; ++i) {
            Eigen::VectorXd acc = ((p.row(i + 2) - p.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                                   (p.row(i + 1) - p.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                                  (u_(i + p_ + 1) - u_(i + 2));
            ;
            bool flag{true};
            for (int j{0}; j < dim; ++j) {
                if (fabs(acc(j)) > acceleration_limit_(j) + 1e-4) {
                    feasible = false;
                    flag = false;
                    break;
                }
            }
            if (flag) continue;
            for (int j{0}; j < dim; ++j)
                max_acc_artio = std::max(max_acc_artio, fabs(acc(j) / acceleration_limit_(j)));

            max_acc_artio = max_acc_artio < artio_limit_ ? max_acc_artio : artio_limit_ + 1e-4;

            // 重新分配时间，也就是重新给节点赋值
            double time_ori = u_(i + p_ + 1) - u_(i + 2);
            double time_new = time_ori * max_acc_artio;
            double delta_t = time_new - time_ori;
            double time_inc = delta_t / double(p_ - 1);

            if (i == 1 || i == 2) {
                for (int j{2}; j < 6; ++j) u_(j) += double(j - 1) * time_inc;
                for (int j{6}; j < u_.rows(); ++j) u_(j) += 4.0 * time_inc;
            } else {
                for (int j{i + 3}; j <= i + p_ + 2; ++j) u_(j) += double(j - i - 2) * time_inc;
                for (int j{i + p_ + 2}; j < u_.rows(); ++j) u_(j) += delta_t;
            }
        }
    }

    return feasible;
}

NonUniformBspline NonUniformBspline::getDerivative(const RobotParams::SharedPtr& robot_params) {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    NonUniformBspline derivative(ctp, p_ - 1, interval_, robot_params);

    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.set_knot(knot);

    return derivative;
}

Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
    Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
    for (int i = 0; i < ctp.rows(); ++i) {
        ctp.row(i) = p_ * (control_points_.row(i + 1) - control_points_.row(i)) /
                     (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
}

double NonUniformBspline::getTimeSum() {
    double tm, tmp;
    getTimeSpan(tm, tmp);
    return tmp - tm;
}

void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
    um = u_(p_);
    um_p = u_(m_ - p_);
}

void NonUniformBspline::setUniformBspline(
    const Eigen::MatrixXd& points, const int& order, const double& interval) {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.rows() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i) {
        if (i <= p_) {
            u_(i) = double(-p_ + i) * interval_;
        } else if (i > p_ && i <= m_ - p_) {
            u_(i) = u_(i - 1) + interval_;
        } else if (i > m_ - p_) {
            u_(i) = u_(i - 1) + interval_;
        }
    }
}

void NonUniformBspline::parameterizeToBspline(
    const double& ts, const std::vector<Eigen::Vector3d>& point_set,
    const std::vector<Eigen::Vector3d>& start_end_derivative, Eigen::MatrixXd& ctrl_pts) {
    if (ts <= 0) {
        std::cout << "[B-spline]:time step error." << std::endl;
        return;
    }

    if (point_set.size() < 2) {
        std::cout << "[B-spline]:point set have only " << point_set.size() << " points."
                  << std::endl;
        return;
    }

    if (start_end_derivative.size() != 4) {
        std::cout << "[B-spline]:derivatives error." << std::endl;
    }

    int K = point_set.size();

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
    // std::cout << "A:\n" << A << std::endl;

    // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
    // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
    // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
    // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i) {
        bx(i) = point_set[i](0);
        by(i) = point_set[i](1);
        bz(i) = point_set[i](2);
    }

    for (int i = 0; i < 4; ++i) {
        bx(K + i) = start_end_derivative[i](0);
        by(K + i) = start_end_derivative[i](1);
        bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    ctrl_pts.resize(K + 2, 3);
    ctrl_pts.col(0) = px;
    ctrl_pts.col(1) = py;
    ctrl_pts.col(2) = pz;

    // std::cout << "[B-spline]: parameterization ok." << std::endl;
}

}  // namespace fast_motion_planning
