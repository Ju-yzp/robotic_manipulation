#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudConverter : public rclcpp::Node {
public:
    PointCloudConverter() : Node("point_cloud_converter") {
        // 订阅PointCloud2话题
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/r1/depth_camera/points", 10,
            std::bind(&PointCloudConverter::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *pcl_cloud);

        RCLCPP_INFO(this->get_logger(), "转换后点云包含 %zu 个点", pcl_cloud->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudConverter>());
    rclcpp::shutdown();
    return 0;
}
