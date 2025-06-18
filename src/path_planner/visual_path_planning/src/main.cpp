// ros2
#include<rclcpp/executors.hpp>
#include<rclcpp/logging.hpp>
#include<rclcpp/node.hpp>
#include<rclcpp/rclcpp.hpp>
#include<rclcpp/publisher.hpp>
#include<rclcpp/utilities.hpp>
#include<visualization_msgs/msg/marker.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<std_msgs/msg/color_rgba.hpp>

// cpp
#include<vector>
#include<thread>
#include<mutex>
#include<atomic>
#include<memory>
#include<chrono>
#include<random>

// 
#include<visual_path_planning/rrt_solver.hpp>

struct Sphere{// 可视化RRT规划路径以及障碍物
float x_;
float y_;
float z_;
float r_;
};

class VisualHelper: public rclcpp::Node
{
public:
VisualHelper():
Node("VisualHelper")
{
marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_result",1);
}

~VisualHelper()
{
    // if(update_thread_.joinable())
    //    update_thread_.join();
}

void set_task(float x,float y,float z,float r,bool flag = false)
{
    visualization_msgs::msg::Marker &marker = flag ? pos_start_ : pos_goal_;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "task";
    marker.id = flag ? 0 : 1;

    // 设置类型
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置球的姿态
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;

    // 设置尺寸
    marker.scale.x = r;
    marker.scale.y = r;
    marker.scale.z = r;

    // 设置线段颜色和尺寸
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

}

// enum SphereType{OBSTACLE,BEGIN,GOAL};
void add_obstacle(float x,float y,float z,float r)
{
    // std::scoped_lock<std::mutex> m_mutex(m_lock_);
    obstacle_infos_.push_back(Sphere{x,y,z,r});

    visualization_msgs::msg::Marker obstacle_marker;
    obstacle_marker.header.frame_id = "map";
    obstacle_marker.header.stamp = this->now();
    obstacle_marker.ns = "obstacle";
    obstacle_marker.id = obstacles_.size();

    // 设置类型
    obstacle_marker.type = visualization_msgs::msg::Marker::SPHERE;
    obstacle_marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置球的姿态
    obstacle_marker.pose.position.x = x;
    obstacle_marker.pose.position.y = y;
    obstacle_marker.pose.position.z = z;
    obstacle_marker.pose.orientation.w = 1.0f;
    obstacle_marker.pose.orientation.x = 0.0f;
    obstacle_marker.pose.orientation.y = 0.0f;
    obstacle_marker.pose.orientation.z = 0.0f;

    // 设置尺寸
    obstacle_marker.scale.x = r;
    obstacle_marker.scale.y = r;
    obstacle_marker.scale.z = r;

    // 设置线段颜色和尺寸
    obstacle_marker.color.r = 0.0f;
    obstacle_marker.color.g = 1.0f;
    obstacle_marker.color.b = 0.0f;
    obstacle_marker.color.a = 0.7f;

    obstacles_.emplace_back(obstacle_marker);
}

template<typename T>
void add_edge(T *start_point,T *end_point)
{
    // 设置类型
    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = this->now();
    edge_marker.ns = "edge";
    edge_marker.id = edges_.size();
    edge_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    edge_marker.action =visualization_msgs::msg::Marker::ADD;

    // 设置线段颜色和尺寸
    edge_marker.scale.x = 0.03f;
    edge_marker.color.r = 0.7f;
    edge_marker.color.g = 0.1f;
    edge_marker.color.b = 0.3f;
    edge_marker.color.a = 1.0f;

    // 设置线段起始点
    geometry_msgs::msg::Point start,end;
    start.x = start_point->get_x();
    start.y = start_point->get_y();
    start.z = start_point->get_z();
    end.x = end_point->get_x();
    end.y = end_point->get_y();
    end.z = end_point->get_z();
    edge_marker.points.push_back(start);
    edge_marker.points.push_back(end);

    edges_.emplace_back(edge_marker);
}

void clear_edges()
{
    edges_.clear();
}

void update()
{
for(auto &obstacle:obstacles_){
    obstacle.header.stamp = this->now();
    marker_pub_->publish(obstacle);
}

for(auto &edge:edges_){
    edge.header.stamp = this->now();
    marker_pub_->publish(edge);
}

pos_start_.header.stamp = this->now();
pos_goal_.header.stamp = this->now();
marker_pub_->publish(pos_start_);
marker_pub_->publish(pos_goal_);
}

// 起始点
visualization_msgs::msg::Marker pos_start_;
visualization_msgs::msg::Marker pos_goal_;

std::vector<Sphere> obstacle_infos_;
private:

// 路径表示的线段
std::vector<visualization_msgs::msg::Marker> edges_;

// 球形障碍物
std::vector<visualization_msgs::msg::Marker> obstacles_;

// 更新线程,频率年为60hz
std::thread update_thread_;

// marker发布者
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

// 多线程资源
std::mutex m_lock_;
std::atomic<bool> flag_;

void start_update_thread()
{
    std::scoped_lock<std::mutex> m_mutex(m_lock_);
    // 内部逻辑：
    // 如果新增一个边，那么将会不发布marker
    // 等待标志变为true后，又开始循环发布，同时每次线程休眠160ms

}

};


bool freeCollision(rrt::Tree &tree,rrt::Node *random,rrt::Node **new_node,VisualHelper &visual_helper)
{
    rrt::Node * nearest_node = tree.get_nearest_node(random);

    // 计算方向向量和长度
    float dx = random->get_x() - nearest_node->get_x();
    float dy = random->get_y() - nearest_node->get_y();
    float dz = random->get_z() - nearest_node->get_z();
    
    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 处理距离过近的情况（小于步长阈值）
    constexpr float min_step = 1e-5f;
    if (distance < min_step) {
        return false;  
    }
    
    // 计算实际扩展步长（不超过0.1）
    constexpr float max_step = 0.1f;
    float step = (distance > max_step) ? max_step : distance;
    float t = step / distance;  // 插值比例
    
    // 计算新节点位置
    float new_x = nearest_node->get_x() + t * dx;
    float new_y = nearest_node->get_y() + t * dy;
    float new_z = nearest_node->get_z() + t * dz;
    
    // 检查所有障碍物
    for (const auto& obs : visual_helper.obstacle_infos_) {
        // 线段向量
        float seg_dx = new_x - nearest_node->get_x();
        float seg_dy = new_y - nearest_node->get_y();
        float seg_dz = new_z - nearest_node->get_z();
        
        // 起点到球心的向量
        float start_to_center_x = obs.x_ - nearest_node->get_x();
        float start_to_center_y = obs.y_ - nearest_node->get_y();
        float start_to_center_z = obs.z_ - nearest_node->get_z();
        
        // 计算线段长度的平方（总是大于0，因为步长固定）
        float seg_length_sq = seg_dx*seg_dx + seg_dy*seg_dy + seg_dz*seg_dz;
        
        // 计算投影比例（由于步长固定，seg_length_sq不会为0）
        float dot_product = start_to_center_x*seg_dx +
                           start_to_center_y*seg_dy +
                           start_to_center_z*seg_dz;
        
        float t_proj = dot_product / seg_length_sq;
        
        // 钳制投影点到线段范围内
        t_proj = std::clamp(t_proj, 0.0f, 1.0f);
        
        // 计算最近点坐标
        float closest_x = nearest_node->get_x() + t_proj * seg_dx;
        float closest_y = nearest_node->get_y() + t_proj * seg_dy;
        float closest_z = nearest_node->get_z() + t_proj * seg_dz;
        
        // 计算最近点到球心的距离平方
        float dist_x = obs.x_ - closest_x;
        float dist_y = obs.y_ - closest_y;
        float dist_z = obs.z_ - closest_z;
        float dist_sq = dist_x*dist_x + dist_y*dist_y + dist_z*dist_z;
        
        // 检查是否相交（比较半径平方）
        if (dist_sq <= obs.r_*obs.r_) 
            return false;  // 与障碍物相交
    }
    
    // 无碰撞，创建新节点
    *new_node = new rrt::Node(new_x, new_y, new_z, nearest_node);
    nearest_node->add_child(*new_node);
    //visual_helper.add_edge(*new_node, nearest_node);
    return true;
}

// 采样函数
rrt::Node* sample(VisualHelper &visual_helper)
{
    std::random_device rd;
    std::mt19937 gen(rd());

    float bound_min_x = -0.3f,bound_min_y = -0.6f,bound_min_z = 0.0f;
    float bound_max_x= 1.6f,bound_max_y = 2.4f,bound_max_z = 3.2f;

      // 获取起点和目标点
    float start_x = visual_helper.pos_start_.pose.position.x;
    float start_y = visual_helper.pos_start_.pose.position.y;
    float start_z = visual_helper.pos_start_.pose.position.z;
    float goal_x = visual_helper.pos_goal_.pose.position.x;
    float goal_y = visual_helper.pos_goal_.pose.position.y;
    float goal_z = visual_helper.pos_goal_.pose.position.z;
    
    // 计算起点到目标的方向向量和距离
    const float dx = goal_x - start_x;
    const float dy = goal_y - start_y;
    const float dz = goal_z - start_z;
    const float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // 决策分布
    std::uniform_real_distribution<float> decision(0.0f, 1.0f);
    const float choice = decision(gen);
    
    // 1. 目标导向采样 (20%概率)
    if (choice < 0.2f) {
        return new rrt::Node(goal_x, goal_y, goal_z);
    }
    
    // 2. 起点附近采样 (15%概率)
    if (choice < 0.35f) {
        // 动态调整采样范围（基于迭代次数）
        static int iteration = 0;
        iteration++;
        
        // 计算最大允许半径（工作空间对角线的10%）
        const float max_radius = 0.1f * std::sqrt(
            (bound_max_x - bound_min_x)*(bound_max_x - bound_min_x) +
            (bound_max_y - bound_min_y)*(bound_max_y - bound_min_y) +
            (bound_max_z - bound_min_z)*(bound_max_z - bound_min_z));
        
        // 基础半径随迭代增加
        float radius = std::min(0.5f + 0.02f * iteration, max_radius);
        
        std::uniform_real_distribution<float> offset(-radius, radius);
        
        return new rrt::Node(
            start_x + offset(gen),
            start_y + offset(gen),
            start_z + offset(gen)
        );
    }
    
    // 3. 路径导向采样 (25%概率)
    if (choice < 0.6f) {
        // 在起点到目标的连线上采样
        std::uniform_real_distribution<float> t_dist(0.0f, 1.0f);
        const float t = t_dist(gen);
        
        // 基础点位置
        const float base_x = start_x + t * dx;
        const float base_y = start_y + t * dy;
        const float base_z = start_z + t * dz;
        
        // 随机偏移量（距离的10-30%）
        const float offset = (0.1f + 0.2f * decision(gen)) * distance;
        
        // 随机方向（球坐标）
        const float theta = 2.0f * M_PI * decision(gen);
        const float phi = M_PI * decision(gen);
        
        // 计算偏移向量
        const float offset_x = offset * std::sin(phi) * std::cos(theta);
        const float offset_y = offset * std::sin(phi) * std::sin(theta);
        const float offset_z = offset * std::cos(phi);
        
        // 确保采样点在工作空间内
        const float x = std::clamp(base_x + offset_x, bound_min_x, bound_max_x);
        const float y = std::clamp(base_y + offset_y, bound_min_y, bound_max_y);
        const float z = std::clamp(base_z + offset_z, bound_min_z, bound_max_z);
        
        return new rrt::Node(x, y, z);
    }
    
    // 4. 目标附近采样 (15%概率)
    if (choice < 0.75f) {
        // 动态调整采样范围
        static int iteration = 0;
        iteration++;
        
        // 计算最大允许半径
        const float max_radius = 0.1f * std::sqrt(
            (bound_max_x - bound_min_x)*(bound_max_x - bound_min_x) +
            (bound_max_y - bound_min_y)*(bound_max_y - bound_min_y) +
            (bound_max_z - bound_min_z)*(bound_max_z - bound_min_z));
        
        float radius = std::min(0.5f + 0.02f * iteration, max_radius);
        
        std::uniform_real_distribution<float> offset(-radius, radius);
        
        return new rrt::Node(
            goal_x + offset(gen),
            goal_y + offset(gen),
            goal_z + offset(gen)
        );
    }
    
    // 5. 工作空间内完全随机采样 (25%概率)
    {
        std::uniform_real_distribution<float> dist_x(bound_min_x, bound_max_x);
        std::uniform_real_distribution<float> dist_y(bound_min_y, bound_max_y);
        std::uniform_real_distribution<float> dist_z(bound_min_z, bound_max_z);
        
        return new rrt::Node(dist_x(gen), dist_y(gen), dist_z(gen));
    }
}

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    std::unique_ptr<VisualHelper> visual_helper = std::make_unique<VisualHelper>();
    visual_helper->add_obstacle(0.5,0.6,0.3,0.35);
    visual_helper->add_obstacle(0.6,1.5,1.4,0.25);
    visual_helper->add_obstacle(0.9,2,1.0,0.45);
    visual_helper->add_obstacle(0.7,1,0.6,0.45);
    visual_helper->add_obstacle(1.0,0.8,0.7,0.3);
    visual_helper->add_obstacle(0.6,1.8,0.8,0.25);
    rrt::Node start(0.1,0.3,0.4),end(1,0.1,2);

    //visual_helper->add_edge(&start, &end);

    //  设置起始点
    visual_helper->set_task(0.2,0.3,0.2,0.1,true);
    visual_helper->set_task(1.2,2.3,0.8,0.1,false);

    rrt::Tree tree;

    rrt::Node *root = new rrt::Node(0.2,0.3,0.2);
    tree.add_node(root);
    // 迭代寻找路径
    const int max_iter = 3000; //最大迭代次数

    for(int count = 0; count < max_iter;count++)
    {
        std::cout<<"!"<<std::endl;
        rrt::Node *random_node = sample(*visual_helper);
        rrt::Node *new_node = nullptr;
        if(freeCollision(tree,random_node,&new_node,*visual_helper))
        {
            tree.add_node(new_node);
            float distance = pow(new_node->get_x() - visual_helper->pos_goal_.pose.position.x,2)+
                             pow(new_node->get_y() - visual_helper->pos_goal_.pose.position.y,2)+
                             pow(new_node->get_z() - visual_helper->pos_goal_.pose.position.z,2);
            
            if(distance < 0.1f)
            {
                RCLCPP_INFO(visual_helper->get_logger(),"Succefully get a safety path ");
                rrt::Node *lastest_node = new rrt::Node(visual_helper->pos_goal_.pose.position.x,
                                                        visual_helper->pos_goal_.pose.position.y,
                                                        visual_helper->pos_goal_.pose.position.z,
                                                   new_node);
                tree.add_node(lastest_node);
                //visual_helper->add_edge(new_node, lastest_node);
                visual_helper->clear_edges(); // 删除所有路径
                // 添加规划好的路径
                auto path = tree.get_path();
                for(int i = path.size()-1;i > 0;i--)
                {
                    visual_helper->add_edge(&path[i-1], &path[i]);
                }
                break;
            }
            std::cout<<"wow"<<std::endl;
            visual_helper->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else continue;
    }

    while (rclcpp::ok()) {
    visual_helper->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rclcpp::shutdown();
}