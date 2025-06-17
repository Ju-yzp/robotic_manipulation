// ros2
#include <chrono>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include<rclcpp/node.hpp>
#include<rclcpp/rclcpp.hpp>
#include<rclcpp/publisher.hpp>
#include<rclcpp/utilities.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include<visualization_msgs/msg/marker.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<std_msgs/msg/color_rgba.hpp>

// cpp
#include<vector>
#include<thread>
#include<mutex>
#include<atomic>
#include<memory>
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

    // obstacle_marker.lifetime = rclcpp::Duration(0,20000000);
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
    edge_marker.scale.x = 0.01f;
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

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    std::unique_ptr<VisualHelper> visual_hepler = std::make_unique<VisualHelper>();
    visual_hepler->add_obstacle(3,5,6,1);
    rrt::Node start(0.1,0.3,0.4),end(1,0.1,2);
    visual_hepler->add_edge(&start, &end);
    //rclcpp::spin(visual_hepler);
    while (rclcpp::ok()) {
    visual_hepler->update();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();
}