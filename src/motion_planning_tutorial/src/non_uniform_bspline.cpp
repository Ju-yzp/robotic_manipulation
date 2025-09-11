// cpp
#include <algorithm>
#include <iostream>
#include <limits>

// motion_palnning_tutorial
#include <motion_planning_tutorial/non_uniform_bspline.hpp>

namespace motion_planning_tutorial {

NonUniformBspline::NonUniformBspline(
    const Eigen::MatrixXd& control_points, const int order, const double& interval,
    const RobotDescription::SharedPtr& robot_description,
    const std::unordered_map<size_t, std::string>& id_map)
    : control_points_(control_points),
      p_(order),
      n_(control_points.rows() - 1),
      m_(n_ + p_ + 1),
      u_(Eigen::VectorXd::Zero(m_ + 1)),
      interval_(interval),
      robot_description_(robot_description),
      id_map_(id_map) {
    for (int i = 0; i <= m_; ++i) {
        if (i <= p_) {
            u_(i) = double(-p_ + i) * interval_;
        } else if (i > p_ && i <= m_ - p_) {
            u_(i) = u_(i - 1) + interval_;
        } else if (i > m_ - p_) {
            u_(i) = u_(i - 1) + interval_;
        }
    }

    for (int i{0}; i < id_map_.size(); ++i) {
        const auto joint_limit = robot_description_->get_jointlimit(id_map_[i]);
        velocity_limit_.conservativeResize(velocity_limit_.size() + 1);
        velocity_limit_(i) = joint_limit.joint_velocity;
        acceleration_limit_.conservativeResize(acceleration_limit_.size() + 1);
        acceleration_limit_(i) = joint_limit.joint_angular_acceleration;
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

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true) {
        if (u_(k + 1) >= ub) break;
        ++k;
    }

    /* deBoor's alg */
    std::vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i) {
        d.push_back(control_points_.row(k - p_ + i));
        // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r) {
        for (int i = p_; i >= r; --i) {
            double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
            // cout << "alpha: " << alpha << endl;
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

NonUniformBspline NonUniformBspline::getDerivative() {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    NonUniformBspline derivative(ctp, p_ - 1, interval_, robot_description_, id_map_);

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
}  // namespace motion_planning_tutorial
