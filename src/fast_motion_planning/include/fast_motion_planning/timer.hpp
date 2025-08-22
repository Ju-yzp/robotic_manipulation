#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <iostream>
#include <chrono>  // 仅依赖时间库

class IntervalTimer {
private:
    // 高精度时间点类型
    using TimePoint = std::chrono::high_resolution_clock::time_point;
    TimePoint start_;  // 仅保存开始时间

public:
    // 记录当前时间作为开始点
    void recordStart() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    // 计算从开始到现在的间隔时间（返回毫秒）
    double getIntervalMs() const {
        auto now = std::chrono::high_resolution_clock::now();
        // 计算间隔并转换为毫秒
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_);
        return duration.count();
    }

    // 计算从开始到现在的间隔时间（返回秒）
    double getIntervalSec() const {
        auto now = std::chrono::high_resolution_clock::now();
        // 计算间隔并转换为秒（带小数）
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_);
        return duration.count();
    }
};

#endif