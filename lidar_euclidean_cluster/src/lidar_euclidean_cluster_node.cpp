#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <geometry_msgs/msg/point.hpp>

using std::placeholders::_1;

class LidarEuclideanClusterNode : public rclcpp::Node
{
public:
  LidarEuclideanClusterNode()
  : Node("lidar_euclidean_cluster_node")
  {
    // -------------------- PARAMETERS --------------------
    this->declare_parameter("input_topic", "/filtered_points");
    this->declare_parameter("output_topic", "/cluster_points");
    this->declare_parameter("marker_topic", "/cluster_markers");
    this->declare_parameter("cluster_tolerance", 0.5);
    this->declare_parameter("min_cluster_size", 30);
    this->declare_parameter("max_cluster_size", 5000);
    this->declare_parameter("publish_markers", true);
    this->declare_parameter("verbose", false);

    this->get_parameter("cluster_tolerance", cluster_tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("marker_topic", marker_topic_);
    this->get_parameter("publish_markers", publish_markers_);
    this->get_parameter("verbose", verbose_);

    // -------------------- SUBSCRIBER --------------------
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, 10,
        std::bind(&LidarEuclideanClusterNode::pointCloudCallback, this, _1));

    // -------------------- PUBLISHERS --------------------
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    if (publish_markers_)
    {
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
    }

    RCLCPP_INFO(this->get_logger(), "Euclidean Cluster Node Started");
    RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
    if (publish_markers_)
    {
      RCLCPP_INFO(this->get_logger(), "  Marker topic: %s", marker_topic_.c_str());
    }
  }

private:

  // Helper function to create wireframe bounding box marker
  visualization_msgs::msg::Marker createBoundingBoxMarker(
    const pcl::PointXYZ& min_pt, const pcl::PointXYZ& max_pt,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp, int cluster_id)
  {
    visualization_msgs::msg::Marker bbox_marker;
    bbox_marker.header.frame_id = frame_id;
    bbox_marker.header.stamp = stamp;
    bbox_marker.ns = "cluster_bboxes";
    bbox_marker.id = cluster_id;
    bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    bbox_marker.action = visualization_msgs::msg::Marker::ADD;
    bbox_marker.pose.orientation.w = 1.0;
    bbox_marker.scale.x = 0.05;  // Line width
    bbox_marker.color.r = 0.0;
    bbox_marker.color.g = 1.0;
    bbox_marker.color.b = 0.0;
    bbox_marker.color.a = 1.0;

    // Define the 8 vertices of the bounding box
    geometry_msgs::msg::Point vertices[8];
    vertices[0].x = min_pt.x; vertices[0].y = min_pt.y; vertices[0].z = min_pt.z;  // min-min-min
    vertices[1].x = max_pt.x; vertices[1].y = min_pt.y; vertices[1].z = min_pt.z;  // max-min-min
    vertices[2].x = max_pt.x; vertices[2].y = max_pt.y; vertices[2].z = min_pt.z;  // max-max-min
    vertices[3].x = min_pt.x; vertices[3].y = max_pt.y; vertices[3].z = min_pt.z;  // min-max-min
    vertices[4].x = min_pt.x; vertices[4].y = min_pt.y; vertices[4].z = max_pt.z;  // min-min-max
    vertices[5].x = max_pt.x; vertices[5].y = min_pt.y; vertices[5].z = max_pt.z;  // max-min-max
    vertices[6].x = max_pt.x; vertices[6].y = max_pt.y; vertices[6].z = max_pt.z;  // max-max-max
    vertices[7].x = min_pt.x; vertices[7].y = max_pt.y; vertices[7].z = max_pt.z;  // min-max-max

    // Draw 12 edges of the box (each edge connects two vertices)
    // Bottom face (z = min)
    bbox_marker.points.push_back(vertices[0]); bbox_marker.points.push_back(vertices[1]);  // edge 0-1
    bbox_marker.points.push_back(vertices[1]); bbox_marker.points.push_back(vertices[2]);  // edge 1-2
    bbox_marker.points.push_back(vertices[2]); bbox_marker.points.push_back(vertices[3]);  // edge 2-3
    bbox_marker.points.push_back(vertices[3]); bbox_marker.points.push_back(vertices[0]);  // edge 3-0
    // Top face (z = max)
    bbox_marker.points.push_back(vertices[4]); bbox_marker.points.push_back(vertices[5]);  // edge 4-5
    bbox_marker.points.push_back(vertices[5]); bbox_marker.points.push_back(vertices[6]);  // edge 5-6
    bbox_marker.points.push_back(vertices[6]); bbox_marker.points.push_back(vertices[7]);  // edge 6-7
    bbox_marker.points.push_back(vertices[7]); bbox_marker.points.push_back(vertices[4]);  // edge 7-4
    // Vertical edges connecting bottom to top
    bbox_marker.points.push_back(vertices[0]); bbox_marker.points.push_back(vertices[4]);  // edge 0-4
    bbox_marker.points.push_back(vertices[1]); bbox_marker.points.push_back(vertices[5]);  // edge 1-5
    bbox_marker.points.push_back(vertices[2]); bbox_marker.points.push_back(vertices[6]);  // edge 2-6
    bbox_marker.points.push_back(vertices[3]); bbox_marker.points.push_back(vertices[7]);  // edge 3-7

    return bbox_marker;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (verbose_)
    {
      RCLCPP_DEBUG(this->get_logger(), "Received cloud with %ld points", cloud->points.size());
    }

    if (cloud->empty())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "Received empty cloud!");
      // Publish empty cloud to maintain pipeline
      sensor_msgs::msg::PointCloud2 output;
      output.header = msg->header;
      pub_->publish(output);
      
      // Delete all existing markers
      if (publish_markers_ && marker_pub_)
      {
        visualization_msgs::msg::MarkerArray delete_markers;
        for (int i = 1; i <= max_cluster_id_seen_; i++)
        {
          // Delete cluster center marker
          visualization_msgs::msg::Marker center_marker;
          center_marker.header.frame_id = msg->header.frame_id;
          center_marker.header.stamp = msg->header.stamp;
          center_marker.ns = "cluster_centers";
          center_marker.id = i;
          center_marker.action = visualization_msgs::msg::Marker::DELETE;
          delete_markers.markers.push_back(center_marker);
          
          // Delete bounding box marker
          visualization_msgs::msg::Marker bbox_marker;
          bbox_marker.header.frame_id = msg->header.frame_id;
          bbox_marker.header.stamp = msg->header.stamp;
          bbox_marker.ns = "cluster_bboxes";
          bbox_marker.id = i;
          bbox_marker.action = visualization_msgs::msg::Marker::DELETE;
          delete_markers.markers.push_back(bbox_marker);
        }
        marker_pub_->publish(delete_markers);
        max_cluster_id_seen_ = 0;  // Reset since all markers are deleted
      }
      return;
    }

    // KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Create output cloud with cluster IDs stored in intensity field
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    clustered_cloud->reserve(cloud->points.size());

    // Visualization markers
    visualization_msgs::msg::MarkerArray marker_array;
    int cluster_id = 1;

    // Process each cluster
    for (const auto& indices : cluster_indices)
    {
      // Extract cluster points and assign cluster ID to intensity
      for (int index : indices.indices)
      {
        const auto& pt = cloud->points[index];
        pcl::PointXYZI pt_i;
        pt_i.x = pt.x;
        pt_i.y = pt.y;
        pt_i.z = pt.z;
        pt_i.intensity = static_cast<float>(cluster_id);
        clustered_cloud->points.push_back(pt_i);
      }

      // Compute cluster centroid and bounding box for visualization
      if (publish_markers_ && marker_pub_ && !indices.indices.empty())
      {
        pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
        cluster_cloud.reserve(indices.indices.size());
        for (int idx : indices.indices)
        {
          cluster_cloud.points.push_back(cloud->points[idx]);
        }
        cluster_cloud.width = cluster_cloud.points.size();
        cluster_cloud.height = 1;
        cluster_cloud.is_dense = true;

        // Compute centroid manually (average of all points)
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (const auto& pt : cluster_cloud.points)
        {
          sum_x += pt.x;
          sum_y += pt.y;
          sum_z += pt.z;
        }
        size_t num_points = cluster_cloud.points.size();
        double centroid_x = sum_x / num_points;
        double centroid_y = sum_y / num_points;
        double centroid_z = sum_z / num_points;

        // Compute bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(cluster_cloud, min_pt, max_pt);

        // Add marker for cluster center
        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = msg->header.frame_id;
        center_marker.header.stamp = msg->header.stamp;
        center_marker.ns = "cluster_centers";
        center_marker.id = cluster_id;
        center_marker.type = visualization_msgs::msg::Marker::SPHERE;
        center_marker.action = visualization_msgs::msg::Marker::ADD;
        center_marker.pose.position.x = centroid_x;
        center_marker.pose.position.y = centroid_y;
        center_marker.pose.position.z = centroid_z;
        center_marker.pose.orientation.w = 1.0;
        center_marker.scale.x = 0.3;
        center_marker.scale.y = 0.3;
        center_marker.scale.z = 0.3;
        center_marker.color.r = 1.0;
        center_marker.color.g = 0.0;
        center_marker.color.b = 0.0;
        center_marker.color.a = 0.8;
        marker_array.markers.push_back(center_marker);

        // Add wireframe bounding box marker (LINE_LIST)
        visualization_msgs::msg::Marker bbox_marker = createBoundingBoxMarker(
          min_pt, max_pt, msg->header.frame_id, msg->header.stamp, cluster_id);
        marker_array.markers.push_back(bbox_marker);
      }

      cluster_id++;
    }

    // Track the maximum cluster ID used (cluster_id is incremented after use, so subtract 1)
    int current_max_cluster_id = (cluster_id > 1) ? (cluster_id - 1) : 0;

    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;

    // Publish clustered cloud with intensity = cluster ID
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*clustered_cloud, output);
    output.header = msg->header;
    pub_->publish(output);
    
    if (verbose_)
    {
      RCLCPP_DEBUG(this->get_logger(), "Published clustered cloud with %ld points to %s",
                  clustered_cloud->points.size(), output_topic_.c_str());
    }

    // Publish visualization markers
    if (publish_markers_ && marker_pub_)
    {
      
      // Delete markers for clusters that no longer exist
      for (int i = current_max_cluster_id + 1; i <= max_cluster_id_seen_; i++)
      {
        // Delete cluster center marker
        visualization_msgs::msg::Marker center_marker;
        center_marker.header.frame_id = msg->header.frame_id;
        center_marker.header.stamp = msg->header.stamp;
        center_marker.ns = "cluster_centers";
        center_marker.id = i;
        center_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(center_marker);
        
        // Delete bounding box marker
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header.frame_id = msg->header.frame_id;
        bbox_marker.header.stamp = msg->header.stamp;
        bbox_marker.ns = "cluster_bboxes";
        bbox_marker.id = i;
        bbox_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(bbox_marker);
      }
      
      // Update maximum cluster ID seen
      max_cluster_id_seen_ = current_max_cluster_id;
      
      marker_pub_->publish(marker_array);
    }

    if (verbose_ || cluster_indices.size() > 0)
    {
      RCLCPP_INFO(this->get_logger(), "Clusters Found: %ld | Points: %ld",
                  cluster_indices.size(),
                  clustered_cloud->points.size());
    }
  }

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Params
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  bool publish_markers_;
  bool verbose_;
  
  // Track maximum cluster ID to delete old markers
  int max_cluster_id_seen_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarEuclideanClusterNode>());
  rclcpp::shutdown();
  return 0;
}

