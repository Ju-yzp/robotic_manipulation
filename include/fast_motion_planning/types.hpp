#ifndef FAST_MOTION_PLANNING_TYPES_HPP_
#define FAST_MOTION_PLANNING_TYPES_HPP_

// eigen
#include <Eigen/Eigen>

// cpp
#include <memory>

using State = Eigen::VectorXd;

// 注册类的智能指针宏
#define REGISTER_SMART_POINTER(registered_class)                    \
    using SharedPtr = std::shared_ptr<registered_class>;            \
    using UniquePtr = std::unique_ptr<registered_class>;            \
    using ConstSharedPtr = std::shared_ptr<const registered_class>; \
    using ConstUniquePtr = std::unique_ptr<const registered_class>;

#endif
