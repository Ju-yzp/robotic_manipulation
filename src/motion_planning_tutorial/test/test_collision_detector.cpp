#include <motion_planning_tutorial/ikd_Tree.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <motion_planning_tutorial/robot_description.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("point_cloud_converter") {
        kd_tree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>();
        // 订阅PointCloud2话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/r1/depth_camera/points", 10,
            std::bind(&PointCloudConverter::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *pcl_cloud);
        kd_tree_->Build(pcl_cloud->points);
        std::cout << (int)kd_tree_->size() << std::endl;
        // kd_tree_->insert(pcl_cloud->points);
        RCLCPP_INFO(this->get_logger(), "转换后点云包含 %zu 个点", pcl_cloud->size());
    }

    std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree_;
    // k-d树用于碰撞检测，暂时未使用
    // 后期可以将点云数据存储到k-d树中以便于快速查询和碰撞检测
    // 这里的kd_tree_可以在需要时进行初始化和使用
    // 例如在点云转换后，可以将点云数据插入到k-d_tree_中
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    namespace mpt = motion_planning_tutorial;
    std::unique_ptr<mpt::RobotDescription> robot_description();
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
