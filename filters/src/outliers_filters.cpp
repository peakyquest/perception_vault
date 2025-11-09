#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class OutliersFilters : public rclcpp::Node
{
public:
  OutliersFilters()
  : Node("outliers_filters")
  {
    // Declare parameters
    this->declare_parameter<int>("mean_k", 20);
    this->declare_parameter<double>("stddev_mul_thresh", 1.5);
    this->declare_parameter<bool>("debug", false);

    // Get parameters
    mean_k_ = this->get_parameter("mean_k").as_int();
    stddev_mul_thresh_ = this->get_parameter("stddev_mul_thresh").as_double();
    debug_ = this->get_parameter("debug").as_bool();

    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points/voxel_filter", rclcpp::SensorDataQoS(),
      std::bind(&OutliersFilters::pointcloud_callback, this, std::placeholders::_1));

    // Publishers
    pub_inliers_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points/inliers", 10);
    if (debug_) {
      pub_outliers_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points/outliers", 10);
    }

    RCLCPP_INFO(this->get_logger(), "OutliersFilters node started. MeanK=%d, Stddev=%.2f, Debug=%s",
                mean_k_, stddev_mul_thresh_, debug_ ? "true" : "false");
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k_);
    sor.setStddevMulThresh(stddev_mul_thresh_);
    sor.filter(*cloud_inliers);

    // Convert and publish inliers
    sensor_msgs::msg::PointCloud2 inliers_msg;
    pcl::toROSMsg(*cloud_inliers, inliers_msg);
    inliers_msg.header = msg->header;
    pub_inliers_->publish(inliers_msg);

    // If debug is enabled, also publish outliers
    if (debug_ && pub_outliers_) {
      sor.setNegative(true);  // get outliers instead of inliers
      sor.filter(*cloud_outliers);

      sensor_msgs::msg::PointCloud2 outliers_msg;
      pcl::toROSMsg(*cloud_outliers, outliers_msg);
      outliers_msg.header = msg->header;
      pub_outliers_->publish(outliers_msg);

      RCLCPP_INFO(this->get_logger(),
                  "Published %zu inliers and %zu outliers",
                  cloud_inliers->size(), cloud_outliers->size());
    }
  }

  // Parameters
  int mean_k_;
  double stddev_mul_thresh_;
  bool debug_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_inliers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_outliers_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutliersFilters>());
  rclcpp::shutdown();
  return 0;
}

