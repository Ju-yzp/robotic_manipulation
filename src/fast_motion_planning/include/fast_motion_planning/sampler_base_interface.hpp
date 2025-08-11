#ifndef FAST_MOTION_PLANNING_SAMPLER_BASE_INTERFACE_HPP_
#define FAST_MOTION_PLANNING_SAMPLER_BASE_INTERFACE_HPP_

#include<cstdint>

#include<Eigen/Eigen>

namespace fast_motion_planning {

typedef Eigen::Matrix4d Pose;

class SamplerOption
{
public:

struct Bound{
double min_x;
double max_x;
double min_y;
double max_y;
double min_z;
double max_z;
};

SamplerOption(double step,uint16_t fct,uint16_t sct,double artio):
step_(step),
failed_count_threshold_(fct),
succeful_count_threshold_(sct),
artio_(artio){}

SamplerOption():
step_(25.0f),
failed_count_threshold_(20),
succeful_count_threshold_(20),
artio_(0.2f),
max_iter_(10000.0f){}

double get_step(){ return step_; }

uint16_t get_failed_count_threshold(){ return failed_count_threshold_; }

uint16_t get_succeful_count_threshold(){ return succeful_count_threshold_; }

uint16_t get_max_iter(){  return max_iter_; }

Bound get_bound()const{ return this->bound_; }

void update_step(bool is_increase){
    if(is_increase)
      step_ = step_ * ( 1.0f + artio_ );
    else 
      step_ = step_ * ( 1.0f - artio_) > min_step_ ? step_ * ( 1.0f - artio_) > min_step_ : step_;
    
}

private:
// 步长
double step_;
// 最小的步长
double min_step_;

// TODO:按道理来说，如果是中间路径点附近均匀采样，那么连续超过这个阈值都是无效点(机械臂无法找到能够避障的关节配置)
//      就说明其实这个点附近大概率是不存在有效点的，可以放弃对着一块区域的探索，或者说换一种思路讲，我们可以对这个
//      中间路径点进行邻近搜索，如果存在密集障碍物，那么大概率不可行(狭窄通道属于极端情况)

// 失败次数阈值，超过时会按比例缩小步长
uint16_t failed_count_threshold_;
// 成功次数阈值，超过时会按比例增大步长
uint16_t succeful_count_threshold_;
// 伸长（缩短）的比例
double artio_;
// 采样次数上限
uint16_t max_iter_;
// 采样范围
Bound bound_;
};

class SamplerBaseInterface{
public:
SamplerBaseInterface(SamplerOption &option):
option_(option){}

SamplerBaseInterface()
:option_(SamplerOption())
{}

virtual ~SamplerBaseInterface(){};

// 采样函数
virtual void sample() = 0;

// 重新设置采样器
//virtual void reset() = 0;

protected:
// 采样器配置
SamplerOption option_;
};
}

#endif