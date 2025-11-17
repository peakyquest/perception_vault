#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <algorithm>
#include <limits>

class GroundSegmentation : public rclcpp::Node
{
public:
  GroundSegmentation()
  : Node("ground_segmentation")
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "/points/voxel_filter");
    this->declare_parameter<std::string>("output_topic", "/points/no_ground");
    this->declare_parameter<double>("ground_threshold", 0.2);
    this->declare_parameter<double>("radial_div_num", 60.0);
    this->declare_parameter<double>("concentric_div_num", 30.0);
    this->declare_parameter<double>("max_range", 50.0);
    this->declare_parameter<double>("min_range", 2.0);
    this->declare_parameter<bool>("debug", false);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    ground_threshold_ = this->get_parameter("ground_threshold").as_double();
    radial_div_num_ = this->get_parameter("radial_div_num").as_double();
    concentric_div_num_ = this->get_parameter("concentric_div_num").as_double();
    max_range_ = this->get_parameter("max_range").as_double();
    min_range_ = this->get_parameter("min_range").as_double();
    debug_ = this->get_parameter("debug").as_bool();

    // Subscribers & Publishers
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentation::pointCloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);

    if (debug_) {
      debug_pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_points", 10);
      debug_pub_no_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/no_ground_points", 10);
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/grid_markers", 10);
    }

    RCLCPP_INFO(this->get_logger(),
                "GroundSegmentation started. Threshold: %.2fm, Debug: %s",
                ground_threshold_, debug_ ? "ON" : "OFF");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Received empty cloud! Publishing empty output.");
      // Publish empty cloud to maintain pipeline
      sensor_msgs::msg::PointCloud2 empty_msg;
      empty_msg.header = msg->header;
      pub_->publish(empty_msg);
      return;
    }

    const double radial_step = 2.0 * M_PI / radial_div_num_;
    const double concentric_step = max_range_ / concentric_div_num_;

    std::vector<std::vector<double>> grid_min_z(
      static_cast<size_t>(radial_div_num_),
      std::vector<double>(static_cast<size_t>(concentric_div_num_), std::numeric_limits<double>::max()));

    // Compute minimum Z in each grid cell
    for (const auto &point : cloud->points) {
      double range = std::sqrt(point.x * point.x + point.y * point.y);
      if (range < min_range_ || range > max_range_)
        continue;

      double angle = std::atan2(point.y, point.x);
      if (angle < 0)
        angle += 2.0 * M_PI;

      int radial_idx = static_cast<int>(angle / radial_step);
      int concentric_idx = static_cast<int>(range / concentric_step);

      if (radial_idx >= 0 && radial_idx < radial_div_num_ &&
          concentric_idx >= 0 && concentric_idx < concentric_div_num_) {
        grid_min_z[radial_idx][concentric_idx] =
            std::min(grid_min_z[radial_idx][concentric_idx], static_cast<double>(point.z));
      }
    }

    // Filter and classify
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>);
    int ground_count = 0;

    for (const auto &point : cloud->points) {
      double range = std::sqrt(point.x * point.x + point.y * point.y);
      if (range < min_range_ || range > max_range_) {
        cloud_no_ground->push_back(point);
        continue;
      }

      double angle = std::atan2(point.y, point.x);
      if (angle < 0)
        angle += 2.0 * M_PI;

      int radial_idx = static_cast<int>(angle / radial_step);
      int concentric_idx = static_cast<int>(range / concentric_step);

      if (radial_idx >= 0 && radial_idx < radial_div_num_ &&
          concentric_idx >= 0 && concentric_idx < concentric_div_num_) {
        double ground_height = grid_min_z[radial_idx][concentric_idx];
        double height_above_ground = point.z - ground_height;

        if (height_above_ground > ground_threshold_) {
          cloud_no_ground->push_back(point);
        } else {
          cloud_ground->push_back(point);
          ground_count++;
        }
      }
    }

    // Publish processed data (even if empty to maintain pipeline)
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_no_ground, output);
    output.header = msg->header;
    pub_->publish(output);
    
    // Warn if all points were filtered out
    if (cloud_no_ground->empty() && !cloud->empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "All points filtered out! Input had %zu points.", cloud->size());
    }

    if (debug_) {
      // Publish ground/non-ground clouds
      sensor_msgs::msg::PointCloud2 ground_msg, no_ground_msg;
      pcl::toROSMsg(*cloud_ground, ground_msg);
      pcl::toROSMsg(*cloud_no_ground, no_ground_msg);
      ground_msg.header = msg->header;
      no_ground_msg.header = msg->header;

      debug_pub_ground_->publish(ground_msg);
      debug_pub_no_ground_->publish(no_ground_msg);

      // Publish grid visualization
      visualization_msgs::msg::MarkerArray markers;
      createGridMarkers(markers, msg->header, radial_step, concentric_step);
      marker_pub_->publish(markers);
    }

    if (debug_) {
      RCLCPP_INFO(this->get_logger(),
                  "Points: input=%zu, ground=%d, output=%zu (%.1f%% removed)",
                  cloud->size(), ground_count, cloud_no_ground->size(),
                  100.0 * ground_count / cloud->size());
    }
  }

  void createGridMarkers(visualization_msgs::msg::MarkerArray &markers,
                         const std_msgs::msg::Header &header,
                         double radial_step, double concentric_step)
  {
    markers.markers.clear();
    int id = 0;

    // Radial lines
    for (int i = 0; i < radial_div_num_; ++i) {
      visualization_msgs::msg::Marker line;
      line.header = header;
      line.ns = "radial_lines";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.02;
      line.color.r = 0.0;
      line.color.g = 1.0;
      line.color.b = 0.0;
      line.color.a = 0.3;

      geometry_msgs::msg::Point p1, p2;
      double angle = i * radial_step;
      p1.x = min_range_ * std::cos(angle);
      p1.y = min_range_ * std::sin(angle);
      p2.x = max_range_ * std::cos(angle);
      p2.y = max_range_ * std::sin(angle);
      p1.z = p2.z = 0.0;
      line.points.push_back(p1);
      line.points.push_back(p2);
      markers.markers.push_back(line);
    }

    // Concentric circles
    for (int j = 0; j < concentric_div_num_; ++j) {
      visualization_msgs::msg::Marker circle;
      circle.header = header;
      circle.ns = "concentric_circles";
      circle.id = id++;
      circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
      circle.action = visualization_msgs::msg::Marker::ADD;
      circle.scale.x = 0.02;
      circle.color.r = 0.0;
      circle.color.g = 0.5;
      circle.color.b = 1.0;
      circle.color.a = 0.3;

      double radius = j * concentric_step;
      for (int k = 0; k <= 360; k += 10) {
        geometry_msgs::msg::Point p;
        double rad = k * M_PI / 180.0;
        p.x = radius * std::cos(rad);
        p.y = radius * std::sin(rad);
        p.z = 0.0;
        circle.points.push_back(p);
      }
      markers.markers.push_back(circle);
    }
  }

  // Parameters
  std::string input_topic_, output_topic_;
  double ground_threshold_, radial_div_num_, concentric_div_num_, max_range_, min_range_;
  bool debug_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_ground_, debug_pub_no_ground_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmentation>());
  rclcpp::shutdown();
  return 0;
}

