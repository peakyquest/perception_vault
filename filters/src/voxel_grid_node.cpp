#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "rclcpp/qos.hpp"  // For SensorDataQoS

class VoxelGridNode : public rclcpp::Node
{
public:
  VoxelGridNode()
  : Node("voxel_grid_node")
  {
    // Declare and get parameters
    this->declare_parameter("input_topic", "/points/raw");
    this->declare_parameter("output_topic", "/points/voxel_filter");
    this->declare_parameter("leaf_size_x", 0.1);
    this->declare_parameter("leaf_size_y", 0.1);
    this->declare_parameter("leaf_size_z", 0.1);
    this->declare_parameter("debug", true);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("leaf_size_x", leaf_size_x_);
    this->get_parameter("leaf_size_y", leaf_size_y_);
    this->get_parameter("leaf_size_z", leaf_size_z_);
    this->get_parameter("debug", debug_);

    // Create subscriber (SensorDataQoS for bag/sensor compatibility)
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&VoxelGridNode::pointcloud_callback, this, std::placeholders::_1));

    // Create publisher
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, 10);

    RCLCPP_INFO(this->get_logger(),
                "VoxelGridNode started:\n  Input: %s\n  Output: %s\n  Leaf sizes: x=%.3f, y=%.3f, z=%.3f\n  Debug: %s",
                input_topic_.c_str(), output_topic_.c_str(),
                leaf_size_x_, leaf_size_y_, leaf_size_z_,
                debug_ ? "true" : "false");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      if (debug_) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud — skipping voxel filter.");
      }
      return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter.filter(*filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header = msg->header;

    pub_->publish(output);

    if (debug_) {
      RCLCPP_INFO(this->get_logger(),
                  "Downsampled cloud: %zu → %zu points (%.1f%% reduction)",
                  cloud->size(), filtered->size(),
                  (1.0 - double(filtered->size()) / double(cloud->size())) * 100.0);
    }
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  double leaf_size_x_;
  double leaf_size_y_;
  double leaf_size_z_;
  bool debug_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridNode>());
  rclcpp::shutdown();
  return 0;
}

