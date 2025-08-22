// fast motion planning
#include<fast_motion_planning/status_space_manager.hpp>

// cpp
#include<random> 
#include<cstddef>
#include<iostream>

namespace fmp = fast_motion_planning;

double generate_random_double() {
    // 静态随机数引擎和分布器（避免每次次调用重新初始化）
    static std::random_device rd;  // 用于获取种子
    static std::mt19937 gen(rd()); // 随机数引擎（梅森旋转算法）
    static std::uniform_real_distribution<double> dist(0.0f, 1.0f);  // 均匀分布[0,1)
    
    return dist(gen);
}

void statistic_func (std::vector<Eigen::Vector3d>& points,fmp::SpaceNode * node)
{
// 显示采样点范围
size_t num = points.size();

double min_x = std::numeric_limits<double>::infinity();
double max_x = std::numeric_limits<double>::lowest();
double min_y = std::numeric_limits<double>::infinity();
double max_y = std::numeric_limits<double>::lowest();
double min_z = std::numeric_limits<double>::infinity();
double max_z = std::numeric_limits<double>::lowest();

Eigen::Vector3d average = Eigen::Vector3d::Zero();

std::for_each(points.begin(),points.end(),[&](const auto& point)
{ average += point; });

average /= (double)(num + 1);
sort(points.begin(),points.end(),[=](auto& point1,auto& point2)
{return (point1 - average).norm() > (point2 - average).norm();});

// 剔除部分距离质心较远的点
points.erase(points.begin(),points.begin() + num * 0.25);

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

std::for_each(points.begin(),points.end(),[&](const auto& point)
{
std::cout << point.format(CleanFmt) << std::endl << std::endl;
min_x = point(0) < min_x ? point(0) : min_x;
max_x = point(0) > max_x ? point(0) : max_x;
min_y = point(1) < min_y ? point(1) : min_y;
max_y = point(1) > max_y ? point(1) : max_y;
min_z = point(2) < min_z ? point(2) : min_z;
max_z = point(2) > max_z ? point(2) : max_z;
});

bool x_1 = min_x > node->center(0);
bool x_2 = max_x > node->center(0);
bool y_1 = min_y > node->center(1);
bool y_2 = max_y > node->center(1);
bool z_1 = min_z > node->center(2);
bool z_2 = max_z > node->center(2);

if((x_1&x_2)&&(y_1&y_2)&&(z_1&z_2))
{
std::cout<<"集中分布"<<std::endl;
}
};
int main()
{

fmp::SpaceNode *node = new fmp::SpaceNode();
node->center = Eigen::Vector3d::Zero();
node->x_extent = 100.0;
node->y_extent = 200.0;
node->z_extent = 130.0;

size_t max_iter = 50;
Eigen::Vector3d translation;
for(size_t count{0}; count < max_iter; ++count)
{
translation(0) = node->center(0) + generate_random_double() * node->x_extent;
translation(1) = node->center(1) + generate_random_double() * node->y_extent;
translation(2) = node->center(2) + generate_random_double() * node->z_extent;
node->invalid_points.emplace_back(translation);
}

node->invalid_points.emplace_back(Eigen::Vector3d{-50,100,-10});
node->invalid_points.emplace_back(Eigen::Vector3d{-55,120,-10});
node->invalid_points.emplace_back(Eigen::Vector3d{-50,-100,-10});
node->invalid_points.emplace_back(Eigen::Vector3d{-50,100,-10});
node->invalid_points.emplace_back(Eigen::Vector3d{-50,-100,80});
node->invalid_points.emplace_back(Eigen::Vector3d{-50,90,-80});
node->invalid_points.emplace_back(Eigen::Vector3d{-50,70,-10});
statistic_func(node->invalid_points, node);
}