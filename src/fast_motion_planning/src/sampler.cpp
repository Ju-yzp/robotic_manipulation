// fast motion planning
#include<fast_motion_planning/sampler.hpp>
#include<fast_motion_planning/timer.hpp>

// cpp
#include<limits>
#include<algorithm>
#include<vector>
#include<random>
#include<fstream>
namespace fast_motion_planning {

Sampler::Sampler(StatusSpaceManager::UniquePtr& status_space_manager,const PlanProblem<double>& problem,
                 std::unique_ptr<CollisionDetector>& collision_detector,
                 std::shared_ptr<KinematicSolverBaseInterface> kinematic_solver,
                 double step,size_t max_sample_count)
:status_space_manager_(std::move(status_space_manager)),
problem_(problem),
collision_detector_(std::move(collision_detector)),
kinematic_solver_(kinematic_solver),
step_(step),
max_sample_count_(max_sample_count)
{}

void Sampler::plan()
{
IntervalTimer timer;
timer.recordStart();
Pose start{problem_.get_start_position(),problem_.get_start_oriention()};
std::cout<<"motion planning"<<std::endl;
auto flag = isFreeCollision(start);
if(!std::get<0>(flag)){
   std::cout<<"Failed"<<std::endl;
   return;}

root_ = std::get<1>(flag);
tree_.emplace_back(root_);

size_t iter_count{0};

nearest_node_ = root_;
while(iter_count < max_sample_count_)
{
// 判断与目标位姿是否小于步长(如果在目标位姿附近振荡该怎么处理?)
double dist = (nearest_node_->pose.translation - problem_.get_goal_position()).norm();
if(dist < step_)
{
Pose goal{problem_.get_goal_position(),problem_.get_goal_oriention()};
auto flag = isFreeCollision(start);
if(std::get<0>(flag))
{
problem_.update_status(true);
auto node= std::get<1>(flag);
node->parent = nearest_node_;
tree_.emplace_back(node);
return ;
}
}else{
//采样策略，如何配合状态空间进行高效的启发式生长
Pose random_pose = sample();
auto node = searchNearestNode(random_pose.translation);
double len = (random_pose.translation - node->pose.translation).norm();
random_pose.translation = node->pose.translation + (random_pose.translation - node->pose.translation ) * (step_ / len);
auto result  = isFreeCollision(random_pose);
if(std::get<0>(result)){
status_space_manager_->update(random_pose.translation,true);
auto random_node = std::get<1>(result);
random_node->parent = node;
tree_.emplace_back(random_node);
update_nearest_node(random_node);
}else status_space_manager_->update(random_pose.translation,false);
}
++iter_count;
//std::cout<<(int)iter_count<<std::endl;
}

double time = timer.getIntervalMs();
std::cout<<"Totally spent "<<time<<" ms"<<std::endl;
}

// 核心函数：如何根据生长树的生长情况
Pose Sampler::sample()
{
Pose pose;

// 根据探索进度，选择区域进行采样
auto space = status_space_manager_->get_space(nearest_node_->pose.translation);

auto goal_pos = problem_.get_goal_position();

double dist = (nearest_node_->pose.translation - goal_pos).norm();
if(dist < 120){
   space(0) = (goal_pos(0) + nearest_node_->pose.translation(0)) / 2.0f;
   space(1) = (goal_pos(1) + nearest_node_->pose.translation(1)) / 2.0f;
   space(2) = (goal_pos(2) + nearest_node_->pose.translation(2)) / 2.0f;
   space(3) = dist * 0.4;
   space(4) = dist * 0.3;
   space(5) = dist *  0.3;
}
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> dist_x(space[0] - space[3], space[0] + space[3]);
std::uniform_real_distribution<double> dist_y(space[1] - space[4], space[1] + space[4]);
std::uniform_real_distribution<double> dist_z(space[2] - space[5], space[2] + space[5]);

pose.translation(0) = dist_x(gen);  // x坐标
pose.translation(1) = dist_y(gen);  // y坐标
pose.translation(2) = dist_z(gen);  // z坐标

// 生成随机朝向：单位四元数
// 使用均匀分布生成单位四元数
std::uniform_real_distribution<double> dist_norm(0.0, 1.0);

double u1 = dist_norm(gen);
double u2 = dist_norm(gen) * 2.0 * M_PI;
double u3 = dist_norm(gen) * 2.0 * M_PI;

double sqrt1MinusU1 = sqrt(1.0 - u1);
double sqrtU1 = sqrt(u1);

// 构造单位四元数 (w, x, y, z)
pose.orientation.w() = sqrt1MinusU1 * cos(u3);
pose.orientation.x() = sqrt1MinusU1 * sin(u3);
pose.orientation.y() = sqrtU1 * cos(u2);
pose.orientation.z() = sqrtU1 * sin(u2);

// 确保四元数是单位化的
pose.orientation.normalize();

   std::ofstream outFile("/home/up/motion_planning/python_tool/data.txt", std::ios::app); 
   if (outFile.is_open()) {
        outFile <<pose.translation(0) << " " << pose.translation(1) << " " << pose.translation(2) << std::endl;
    outFile.close();}
return pose;

}

void Sampler::update(Pose& pose,bool is_reached)
{
status_space_manager_->update(pose.translation,is_reached);
}

void Sampler::reset_plan_problem(const PlanProblem<double>& problem)
{
problem_ = problem;
}

void Sampler::releaseMemorySource()
{
for(auto node:tree_)
    if(node)
       delete node;
}

Sampler::RRTNode* Sampler::searchNearestNode(const Eigen::Vector3d&translation)
{
double min_dist = std::numeric_limits<double>::infinity();
RRTNode * nearest_node{nullptr};
std::for_each(tree_.begin(),tree_.end(),[&](const auto& node)
{
double dist = (node->pose.translation - translation).norm();
if(min_dist > dist)
{
nearest_node = node;
min_dist = dist;
}
});

return nearest_node;
}

std::tuple<bool,Sampler::RRTNode *> Sampler::isFreeCollision(Pose& pose)
{
auto solutions = kinematic_solver_->solveInverseKinematic(pose.toMatrix());
solutions.erase(
std::remove_if(solutions.begin(),solutions.end(),
[this](auto solution)
{ 
kinematic_solver_->update_envelopes_position(solution);
return collision_detector_->is_collision_with_obstacles();}),solutions.end());

if(!solutions.empty())
{
RRTNode* node = new RRTNode();
node->pose = pose;
node->valid_solutions = solutions;
return std::tuple<bool,RRTNode *>{true,node};
}
return std::tuple<bool,RRTNode *>{false,nullptr};
}

std::vector<Eigen::VectorXd> Sampler::get_path()
{
if(!problem_.get_status())
   return std::vector<Eigen::VectorXd>();

std::vector<Eigen::VectorXd> path;
auto last_node = tree_[tree_.size() - 1]->parent;
auto last_solution = tree_[tree_.size() - 1]->valid_solutions[0];
path.push_back(last_solution);


while(last_node->parent)
{
if(!weight_func_)
   path.emplace_back(last_node->valid_solutions[0]);
else
{
   std::ofstream outFile("/home/up/motion_planning/python_tool/statistic_data.txt", std::ios::app); 
   if (outFile.is_open()) {
        outFile <<last_node->pose.translation(0) << " " << last_node->pose.translation(1) << " " << last_node->pose.translation(2) << std::endl;
    outFile.close();
}
   last_solution = weight_func_(last_node->valid_solutions,last_solution);
   path.emplace_back(last_solution);
}
last_node = last_node->parent;
}
path.emplace_back(root_->valid_solutions[0]);
return path;
}
}