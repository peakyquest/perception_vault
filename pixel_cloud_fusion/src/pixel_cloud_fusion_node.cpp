#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "yolo_msgs/msg/detection_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>

// Hash function for cv::Point to use as key in unordered_map
namespace std {
  template<>
  struct hash<cv::Point> {
    size_t operator()(const cv::Point& p) const {
      return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
  };
}

class PixelCloudFusionNode : public rclcpp::Node
{
public:
  PixelCloudFusionNode()
  : Node("pixel_cloud_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare parameters
    this->declare_parameter<std::string>("image_topic", "/image_rect");
    this->declare_parameter<std::string>("pointcloud_topic", "/points/raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    this->declare_parameter<std::string>("output_topic", "/points/colored");
    this->declare_parameter<std::string>("camera_frame", "camera_frame");
    this->declare_parameter<std::string>("lidar_frame", "lidar_frame");
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<bool>("use_approximate_sync", true);
    this->declare_parameter<double>("max_range", 100.0);
    this->declare_parameter<std::string>("detections_topic", "/detections");
    this->declare_parameter<std::string>("markers_topic", "/detection_bboxes_3d");
    this->declare_parameter<bool>("enable_3d_bboxes", true);
    this->declare_parameter<double>("bbox_min_height", 0.2);
    this->declare_parameter<double>("bbox_outlier_percentile", 0.85);
    this->declare_parameter<bool>("bbox_remove_ground", true);
    this->declare_parameter<double>("bbox_max_dimension", 10.0);
    this->declare_parameter<bool>("bbox_use_tight_fit", true);

    // Get parameters
    image_topic_ = this->get_parameter("image_topic").as_string();
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    lidar_frame_ = this->get_parameter("lidar_frame").as_string();
    queue_size_ = this->get_parameter("queue_size").as_int();
    use_approximate_sync_ = this->get_parameter("use_approximate_sync").as_bool();
    max_range_ = this->get_parameter("max_range").as_double();
    detections_topic_ = this->get_parameter("detections_topic").as_string();
    markers_topic_ = this->get_parameter("markers_topic").as_string();
    enable_3d_bboxes_ = this->get_parameter("enable_3d_bboxes").as_bool();
    bbox_min_height_ = this->get_parameter("bbox_min_height").as_double();
    bbox_outlier_percentile_ = this->get_parameter("bbox_outlier_percentile").as_double();
    bbox_remove_ground_ = this->get_parameter("bbox_remove_ground").as_bool();
    bbox_max_dimension_ = this->get_parameter("bbox_max_dimension").as_double();
    bbox_use_tight_fit_ = this->get_parameter("bbox_use_tight_fit").as_bool();

    // Create subscribers
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      queue_size_,
      std::bind(&PixelCloudFusionNode::cameraInfoCallback, this, std::placeholders::_1)
    );

    // Create publishers
    colored_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_, queue_size_);
    
    if (enable_3d_bboxes_) {
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        markers_topic_, queue_size_);
    }

    // Create YOLO detections subscriber (using yolo_msgs)
    detections_sub_ = this->create_subscription<yolo_msgs::msg::DetectionArray>(
      detections_topic_,
      queue_size_,
      std::bind(&PixelCloudFusionNode::detectionsCallback, this, std::placeholders::_1)
    );

    // Setup subscribers based on sync mode
    if (use_approximate_sync_) {
      // Message filters initialization will be done in init() method
      // after the node is fully constructed
    } else {
      // Non-synchronized subscribers (will use latest messages)
      image_sub_unsync_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        queue_size_,
        std::bind(&PixelCloudFusionNode::imageCallback, this, std::placeholders::_1)
      );
      
      cloud_sub_unsync_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_,
        queue_size_,
        std::bind(&PixelCloudFusionNode::cloudCallback, this, std::placeholders::_1)
      );
    }

    RCLCPP_INFO(this->get_logger(), "Pixel-Cloud Fusion Node initialized");
    RCLCPP_INFO(this->get_logger(), "  Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  PointCloud topic: %s", pointcloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera info topic: %s", camera_info_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  LiDAR frame: %s", lidar_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Detections topic: %s", detections_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  3D bboxes enabled: %s", enable_3d_bboxes_ ? "true" : "false");
    
    // Initialize sync_ to nullptr
    sync_ = nullptr;
  }

  ~PixelCloudFusionNode()
  {
    if (sync_) {
      delete sync_;
    }
  }

  void init()
  {
    // Initialize message_filters subscribers after node is fully constructed
    if (use_approximate_sync_) {
      // Create message_filters subscribers using shared_from_this()
      image_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        shared_from_this(), image_topic_);
      cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        shared_from_this(), pointcloud_topic_);
      
      sync_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(
        ApproximateSyncPolicy(queue_size_),
        *image_sub_,
        *cloud_sub_
      );
      sync_->registerCallback(
        std::bind(&PixelCloudFusionNode::synchronizedCallback, this,
                  std::placeholders::_1, std::placeholders::_2)
      );
      RCLCPP_INFO(this->get_logger(), "Message filters synchronization enabled");
    }
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::PointCloud2> ApproximateSyncPolicy;

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) {
      return;  // Already received
    }

    // Extract camera matrix
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];  // fx
    camera_matrix_.at<double>(0, 1) = msg->k[1];  // skew
    camera_matrix_.at<double>(0, 2) = msg->k[2];  // cx
    camera_matrix_.at<double>(1, 0) = msg->k[3];  // 0
    camera_matrix_.at<double>(1, 1) = msg->k[4];  // fy
    camera_matrix_.at<double>(1, 2) = msg->k[5];  // cy
    camera_matrix_.at<double>(2, 0) = msg->k[6];  // 0
    camera_matrix_.at<double>(2, 1) = msg->k[7];  // 0
    camera_matrix_.at<double>(2, 2) = msg->k[8];  // 1

    // Extract distortion coefficients
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      dist_coeffs_.at<double>(i) = msg->d[i];
    }

    // Extract projection matrix parameters (P matrix)
    // P[0] = fx, P[2] = cx, P[5] = fy, P[6] = cy
    fx_ = static_cast<float>(msg->p[0]);
    fy_ = static_cast<float>(msg->p[5]);
    cx_ = static_cast<float>(msg->p[2]);
    cy_ = static_cast<float>(msg->p[6]);

    image_width_ = msg->width;
    image_height_ = msg->height;
    camera_info_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Camera info received: %dx%d", image_width_, image_height_);
    RCLCPP_INFO(this->get_logger(), "Camera params: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    latest_image_ = msg;
    processFusion();
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_cloud_ = msg;
    processFusion();
  }

  void synchronizedCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
  {
    latest_image_ = image_msg;
    latest_cloud_ = cloud_msg;
    processFusion();
  }

  std::vector<pcl::PointXYZ> filterBoundingBoxPoints(const std::vector<pcl::PointXYZ>& points)
  {
    if (points.empty()) {
      return points;
    }
    
    std::vector<pcl::PointXYZ> filtered;
    filtered.reserve(points.size());
    
    // Step 1: Remove ground points (points with very low Z relative to the cluster)
    if (bbox_remove_ground_) {
      // Find minimum Z (ground level for this cluster)
      float min_z = std::numeric_limits<float>::max();
      for (const auto &pt : points) {
        min_z = std::min(min_z, pt.z);
      }
      
      // Only keep points above ground threshold
      for (const auto &pt : points) {
        if (pt.z > min_z + bbox_min_height_) {
          filtered.push_back(pt);
        }
      }
    } else {
      filtered = points;
    }
    
    if (filtered.size() < 3) {
      return filtered;  // Too few points to filter further
    }
    
    // Step 2: Compute median (more robust than mean for outlier removal)
    std::vector<float> x_vals, y_vals, z_vals;
    x_vals.reserve(filtered.size());
    y_vals.reserve(filtered.size());
    z_vals.reserve(filtered.size());
    
    for (const auto &pt : filtered) {
      x_vals.push_back(pt.x);
      y_vals.push_back(pt.y);
      z_vals.push_back(pt.z);
    }
    
    std::sort(x_vals.begin(), x_vals.end());
    std::sort(y_vals.begin(), y_vals.end());
    std::sort(z_vals.begin(), z_vals.end());
    
    float median_x = x_vals[x_vals.size() / 2];
    float median_y = y_vals[y_vals.size() / 2];
    float median_z = z_vals[z_vals.size() / 2];
    
    // Step 3: Compute distances from median and find percentile threshold
    std::vector<std::pair<float, size_t>> distance_index_pairs;
    distance_index_pairs.reserve(filtered.size());
    
    for (size_t j = 0; j < filtered.size(); j++) {
      const auto &pt = filtered[j];
      float dx = pt.x - median_x;
      float dy = pt.y - median_y;
      float dz = pt.z - median_z;
      float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      distance_index_pairs.push_back(std::make_pair(distance, j));
    }
    
    // Sort by distance to find percentile
    std::sort(distance_index_pairs.begin(), distance_index_pairs.end(),
              [](const std::pair<float, size_t>& a, const std::pair<float, size_t>& b) {
                return a.first < b.first;
              });
    
    size_t percentile_idx = static_cast<size_t>(bbox_outlier_percentile_ * distance_index_pairs.size());
    if (percentile_idx >= distance_index_pairs.size()) {
      percentile_idx = distance_index_pairs.size() - 1;
    }
    float distance_threshold = distance_index_pairs[percentile_idx].first;
    
    // Also apply maximum dimension constraint
    float max_distance = bbox_max_dimension_ / 2.0f;  // Half dimension (radius from center)
    distance_threshold = std::min(distance_threshold, max_distance);
    
    // Step 4: Filter points within percentile distance
    std::vector<pcl::PointXYZ> final_filtered;
    final_filtered.reserve(filtered.size());
    for (const auto &pair : distance_index_pairs) {
      if (pair.first <= distance_threshold) {
        final_filtered.push_back(filtered[pair.second]);
      }
    }
    
    // Step 5: If using tight fit, apply additional filtering based on axis-aligned bounds
    if (bbox_use_tight_fit_ && final_filtered.size() > 10) {
      // Compute tight bounds on filtered points
      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();
      
      for (const auto &pt : final_filtered) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
      }
      
      // Compute dimensions
      float dim_x = max_x - min_x;
      float dim_y = max_y - min_y;
      float dim_z = max_z - min_z;
      
      // Compute median absolute deviation (MAD) for each axis to find outliers
      std::vector<float> x_mad, y_mad, z_mad;
      float center_x = (min_x + max_x) / 2.0f;
      float center_y = (min_y + max_y) / 2.0f;
      float center_z = (min_z + max_z) / 2.0f;
      
      for (const auto &pt : final_filtered) {
        x_mad.push_back(std::abs(pt.x - center_x) / dim_x);
        y_mad.push_back(std::abs(pt.y - center_y) / dim_y);
        z_mad.push_back(std::abs(pt.z - center_z) / dim_z);
      }
      
      std::sort(x_mad.begin(), x_mad.end());
      std::sort(y_mad.begin(), y_mad.end());
      std::sort(z_mad.begin(), z_mad.end());
      
      float mad_threshold_x = x_mad[static_cast<size_t>(bbox_outlier_percentile_ * x_mad.size())];
      float mad_threshold_y = y_mad[static_cast<size_t>(bbox_outlier_percentile_ * y_mad.size())];
      float mad_threshold_z = z_mad[static_cast<size_t>(bbox_outlier_percentile_ * z_mad.size())];
      
      // Filter points that are outliers in any dimension
      std::vector<pcl::PointXYZ> tight_filtered;
      tight_filtered.reserve(final_filtered.size());
      for (const auto &pt : final_filtered) {
        float mad_x = std::abs(pt.x - center_x) / dim_x;
        float mad_y = std::abs(pt.y - center_y) / dim_y;
        float mad_z = std::abs(pt.z - center_z) / dim_z;
        
        if (mad_x <= mad_threshold_x && mad_y <= mad_threshold_y && mad_z <= mad_threshold_z) {
          tight_filtered.push_back(pt);
        }
      }
      
      // Only use tight filtered if it removed a reasonable amount (not too aggressive)
      if (tight_filtered.size() >= final_filtered.size() * 0.5f) {
        return tight_filtered;
      }
    }
    
    return final_filtered;
  }

  void detectionsCallback(const yolo_msgs::msg::DetectionArray::SharedPtr msg)
  {
    latest_detections_ = msg;
    if (msg->detections.size() > 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Received %zu detections on topic %s", msg->detections.size(), detections_topic_.c_str());
      
      // Log first detection details for debugging
      if (!msg->detections.empty()) {
        const auto &det = msg->detections[0];
        RCLCPP_DEBUG(this->get_logger(), 
          "First detection: center=(%.1f, %.1f), size=(%.1f, %.1f), class=%s",
          det.bbox.center.position.x, det.bbox.center.position.y,
          det.bbox.size.x, det.bbox.size.y,
          !det.class_name.empty() ? det.class_name.c_str() : "unknown");
      }
    }
    
    // Trigger fusion processing when detections arrive (if we have image and cloud)
    if (camera_info_received_ && latest_image_ && latest_cloud_) {
      processFusion();
    } else {
      RCLCPP_DEBUG(this->get_logger(), 
        "Detections received but waiting for camera_info=%d, image=%d, cloud=%d",
        camera_info_received_, latest_image_ != nullptr, latest_cloud_ != nullptr);
    }
  }

  void processFusion()
  {
    if (!camera_info_received_ || !latest_image_ || !latest_cloud_) {
      return;
    }

    try {
      // Get transform from LiDAR to camera frame
      geometry_msgs::msg::TransformStamped transform;
      try {
        transform = tf_buffer_.lookupTransform(
          camera_frame_,
          latest_cloud_->header.frame_id,
          latest_cloud_->header.stamp,
          rclcpp::Duration::from_seconds(0.1)
        );
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Could not transform from %s to %s: %s",
          latest_cloud_->header.frame_id.c_str(), camera_frame_.c_str(), ex.what());
        return;
      }

      // Convert transform to Eigen
      Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform.transform);
      Eigen::Matrix4f transform_matrix = transform_eigen.matrix().cast<float>();

      // Convert ROS image to OpenCV
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(latest_image_, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Convert point cloud from ROS to PCL
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*latest_cloud_, *input_cloud);

      if (input_cloud->empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Received empty point cloud");
        return;
      }

      // Transform all points to camera frame and create projection map
      // Map from image pixel (u, v) to original 3D point in LiDAR frame
      std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;
      
      // First pass: Transform points to camera frame and project to image
      for (const auto &point : input_cloud->points) {
        // Transform point to camera frame
        Eigen::Vector4f point_lidar(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f point_camera = transform_matrix * point_lidar;

        // Skip points behind camera or too far
        if (point_camera.z() <= 0 || point_camera.z() > max_range_) {
          continue;
        }

        // Simple pinhole projection (for rectified images)
        // u = x * fx / z + cx
        // v = y * fy / z + cy
        int u = static_cast<int>(point_camera.x() * fx_ / point_camera.z() + cx_);
        int v = static_cast<int>(point_camera.y() * fy_ / point_camera.z() + cy_);

        // Only add points that project to valid image coordinates
        if (u >= 0 && u < image_width_ && v >= 0 && v < image_height_) {
          projection_map[cv::Point(u, v)] = point;
        }
      }

      // Create a set to mark pixels that are within detection bounding boxes
      std::unordered_set<cv::Point, std::hash<cv::Point>> detected_pixels;
      std::vector<std::pair<cv::Rect, std::string>> detection_boxes;
      
      if (latest_detections_) {
        RCLCPP_DEBUG(this->get_logger(), "Processing %zu detections", latest_detections_->detections.size());
        for (const auto &detection : latest_detections_->detections) {
          // Get bounding box (yolo_msgs uses bbox.size.x/y instead of size_x/size_y)
          double center_x = detection.bbox.center.position.x;
          double center_y = detection.bbox.center.position.y;
          double size_x = detection.bbox.size.x;
          double size_y = detection.bbox.size.y;
          
          // Calculate bounding box bounds
          int x_min = static_cast<int>(center_x - size_x / 2.0);
          int y_min = static_cast<int>(center_y - size_y / 2.0);
          int x_max = static_cast<int>(center_x + size_x / 2.0);
          int y_max = static_cast<int>(center_y + size_y / 2.0);
          
          // Clamp to image bounds
          x_min = std::max(0, std::min(x_min, image_width_ - 1));
          y_min = std::max(0, std::min(y_min, image_height_ - 1));
          x_max = std::max(0, std::min(x_max, image_width_ - 1));
          y_max = std::max(0, std::min(y_max, image_height_ - 1));
          
          // Get class name (yolo_msgs has class_name directly)
          std::string class_name = !detection.class_name.empty() ? detection.class_name : "object";
          
          // Mark all pixels within bounding box
          for (int row = y_min; row <= y_max; row++) {
            for (int col = x_min; col <= x_max; col++) {
              detected_pixels.insert(cv::Point(col, row));
            }
          }
          
          // Store bounding box info for 3D marker creation
          detection_boxes.push_back(std::make_pair(
            cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min),
            class_name
          ));
        }
      }

      // Second pass: Iterate through image pixels and create colored point cloud
      // Only include points that have corresponding image pixels
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      colored_cloud->points.clear();
      colored_cloud->is_dense = false;

      // Map to store points for each detection (for 3D bounding box computation)
      std::vector<std::vector<pcl::PointXYZ>> detection_points;
      if (enable_3d_bboxes_ && latest_detections_) {
        detection_points.resize(latest_detections_->detections.size());
      }

      for (int row = 0; row < image_height_; row++) {
        for (int col = 0; col < image_width_; col++) {
          cv::Point pixel(col, row);
          auto it = projection_map.find(pixel);
          
          if (it != projection_map.end()) {
            // Found a 3D point that projects to this pixel
            pcl::PointXYZRGB colored_point;
            const pcl::PointXYZ &corresponding_3d_point = it->second;
            
            // Check if this pixel is within a detection bounding box
            bool is_detected = detected_pixels.find(pixel) != detected_pixels.end();
            
            // Get RGB from image (BGR format in OpenCV)
            cv::Vec3b rgb_pixel = cv_ptr->image.at<cv::Vec3b>(row, col);
            
            // Set point coordinates (in original LiDAR frame)
            colored_point.x = corresponding_3d_point.x;
            colored_point.y = corresponding_3d_point.y;
            colored_point.z = corresponding_3d_point.z;
            
            if (is_detected) {
              // Highlight detected objects with bright red/yellow color
              colored_point.r = 255;
              colored_point.g = 0;
              colored_point.b = 0;
              
              // Store point for 3D bounding box computation
              if (enable_3d_bboxes_ && latest_detections_) {
                // Find which detection this pixel belongs to
                for (size_t i = 0; i < detection_boxes.size(); i++) {
                  const auto &bbox = detection_boxes[i].first;
                  if (col >= bbox.x && col <= bbox.x + bbox.width &&
                      row >= bbox.y && row <= bbox.y + bbox.height) {
                    detection_points[i].push_back(corresponding_3d_point);
                    break;
                  }
                }
              }
            } else {
              // Set RGB color from image (BGR to RGB conversion)
              colored_point.r = rgb_pixel[2];
              colored_point.g = rgb_pixel[1];
              colored_point.b = rgb_pixel[0];
            }
            
            colored_cloud->points.push_back(colored_point);
          }
        }
      }
      
      // Set point cloud properties
      colored_cloud->width = colored_cloud->points.size();
      colored_cloud->height = 1;

      // Convert colored point cloud back to ROS message
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*colored_cloud, output_msg);
      output_msg.header = latest_cloud_->header;

      // Publish colored point cloud
      colored_cloud_pub_->publish(output_msg);

      // Publish 3D bounding box markers if enabled
      if (enable_3d_bboxes_ && marker_pub_ && latest_detections_) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Delete old markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        
        RCLCPP_DEBUG(this->get_logger(), "Processing %zu detections for 3D bbox creation", 
                     latest_detections_->detections.size());
        
        // Create markers for each detection
        int markers_created = 0;
        for (size_t i = 0; i < detection_points.size() && i < latest_detections_->detections.size(); i++) {
          if (detection_points[i].empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Detection %zu has no 3D points", i);
            continue;
          }
          
          RCLCPP_DEBUG(this->get_logger(), "Detection %zu has %zu 3D points", i, detection_points[i].size());
          
          // Filter points to get tighter bounding box
          std::vector<pcl::PointXYZ> filtered_points = filterBoundingBoxPoints(detection_points[i]);
          
          if (filtered_points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Detection %zu: all points filtered out", i);
            continue;
          }
          
          RCLCPP_DEBUG(this->get_logger(), "Detection %zu: filtered from %zu to %zu points", 
                       i, detection_points[i].size(), filtered_points.size());
          
          // Compute 3D bounding box from filtered points
          // Use exact min/max without any padding - this ensures the box fits tightly around the points
          pcl::PointXYZ min_pt, max_pt;
          min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
          max_pt.x = max_pt.y = max_pt.z = std::numeric_limits<float>::lowest();
          
          for (const auto &pt : filtered_points) {
            min_pt.x = std::min(min_pt.x, pt.x);
            min_pt.y = std::min(min_pt.y, pt.y);
            min_pt.z = std::min(min_pt.z, pt.z);
            max_pt.x = std::max(max_pt.x, pt.x);
            max_pt.y = std::max(max_pt.y, pt.y);
            max_pt.z = std::max(max_pt.z, pt.z);
          }
          
          // Ensure we have valid dimensions
          if (max_pt.x <= min_pt.x || max_pt.y <= min_pt.y || max_pt.z <= min_pt.z) {
            RCLCPP_WARN(this->get_logger(), "Detection %zu: invalid bounding box dimensions, skipping", i);
            continue;
          }
          
          // Create bounding box marker
          visualization_msgs::msg::Marker bbox_marker;
          bbox_marker.header.frame_id = latest_cloud_->header.frame_id;
          bbox_marker.header.stamp = latest_cloud_->header.stamp;
          bbox_marker.ns = "detection_bboxes";
          bbox_marker.id = static_cast<int>(i);
          bbox_marker.type = visualization_msgs::msg::Marker::CUBE;
          bbox_marker.action = visualization_msgs::msg::Marker::ADD;
          bbox_marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
          bbox_marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
          bbox_marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
          bbox_marker.pose.orientation.w = 1.0;
          // Compute dimensions exactly from filtered points (no padding)
          double dim_x = max_pt.x - min_pt.x;
          double dim_y = max_pt.y - min_pt.y;
          double dim_z = max_pt.z - min_pt.z;
          
          // Use exact dimensions - only apply minimum if really needed (very small boxes)
          // This ensures the box tightly fits the point cloud
          bbox_marker.scale.x = dim_x > 0.05 ? dim_x : 0.05;
          bbox_marker.scale.y = dim_y > 0.05 ? dim_y : 0.05;
          bbox_marker.scale.z = dim_z > 0.05 ? dim_z : 0.05;
          
          // Log bounding box size for debugging
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Detection %zu '%s': bbox size=(%.2f, %.2f, %.2f)m, points=%zu->%zu, center=(%.2f, %.2f, %.2f)",
            i, detection_boxes[i].second.c_str(), dim_x, dim_y, dim_z,
            detection_points[i].size(), filtered_points.size(),
            bbox_marker.pose.position.x, bbox_marker.pose.position.y, bbox_marker.pose.position.z);
          bbox_marker.color.r = 1.0;
          bbox_marker.color.g = 0.0;
          bbox_marker.color.b = 0.0;
          bbox_marker.color.a = 0.3;
          bbox_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
          
          marker_array.markers.push_back(bbox_marker);
          
          // Create text marker for class label
          visualization_msgs::msg::Marker text_marker;
          text_marker.header = bbox_marker.header;
          text_marker.ns = "detection_labels";
          text_marker.id = static_cast<int>(i);
          text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          text_marker.action = visualization_msgs::msg::Marker::ADD;
          text_marker.pose.position.x = bbox_marker.pose.position.x;
          text_marker.pose.position.y = bbox_marker.pose.position.y;
          text_marker.pose.position.z = max_pt.z + 0.5;
          text_marker.pose.orientation.w = 1.0;
          text_marker.scale.z = 1.0;  // Increased from 0.5 for better visibility in RViz
          text_marker.color.r = 1.0;
          text_marker.color.g = 1.0;
          text_marker.color.b = 1.0;
          text_marker.color.a = 1.0;
          text_marker.text = detection_boxes[i].second;
          text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
          
          marker_array.markers.push_back(text_marker);
          markers_created++;
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Published %d 3D bounding box markers from %zu detections (total points in detection boxes: %zu)", 
          markers_created, latest_detections_->detections.size(), detected_pixels.size());
        
        // Always publish markers array (even if empty, to clear old markers)
        marker_pub_->publish(marker_array);
        
        if (markers_created == 0 && latest_detections_->detections.size() > 0) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "No 3D markers created for %zu detections - no points found within bounding boxes. "
            "Check if LiDAR points project into detection regions.", 
            latest_detections_->detections.size());
        }
      } else if (enable_3d_bboxes_ && marker_pub_ && !latest_detections_) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "3D bboxes enabled but no detections received yet");
      } else if (enable_3d_bboxes_ && !marker_pub_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "3D bboxes enabled but marker publisher not initialized");
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error in fusion process: %s", e.what());
    }
  }

  // Parameters
  std::string image_topic_;
  std::string pointcloud_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string camera_frame_;
  std::string lidar_frame_;
  int queue_size_;
  bool use_approximate_sync_;
  double max_range_;
  std::string detections_topic_;
  std::string markers_topic_;
  bool enable_3d_bboxes_;
  double bbox_min_height_;
  double bbox_outlier_percentile_;
  bool bbox_remove_ground_;
  double bbox_max_dimension_;
  bool bbox_use_tight_fit_;

  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_unsync_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_unsync_;
  rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detections_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Synchronized subscribers
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> cloud_sub_;
  message_filters::Synchronizer<ApproximateSyncPolicy> * sync_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Camera parameters
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  float fx_ = 0.0f;
  float fy_ = 0.0f;
  float cx_ = 0.0f;
  float cy_ = 0.0f;
  int image_width_ = 0;
  int image_height_ = 0;
  bool camera_info_received_ = false;

  // Latest messages
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  yolo_msgs::msg::DetectionArray::SharedPtr latest_detections_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PixelCloudFusionNode>();
  node->init();  // Initialize message_filters after node construction
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
