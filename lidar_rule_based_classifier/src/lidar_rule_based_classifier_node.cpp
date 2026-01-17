#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>

using std::placeholders::_1;

// Structure to store extracted features for a cluster
struct ClusterFeatures
{
  int cluster_id;
  std::vector<pcl::PointXYZ> points;
  
  // Geometric features
  double height = 0.0;  // Z-extent
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();  // (x, y, z)
  struct BoundingBox {
    double length = 0.0;
    double width = 0.0;
    double height = 0.0;
  } bounding_box;
  double volume = 0.0;
  double aspect_ratio = 0.0;  // length / width
  double planarity = 0.0;
  double linearity = 0.0;
  double sphericity = 0.0;
  
  // Statistical features
  double mean_height = 0.0;
  double std_height = 0.0;
  double point_density = 0.0;
  size_t num_points = 0;
  
  // Distance from sensor origin
  double distance_from_origin = 0.0;  // Distance in meters
  
  // Eigenvalue-based features
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
  Eigen::Vector2d eigenvalue_ratios = Eigen::Vector2d::Zero();
};

class LidarRuleBasedClassifierNode : public rclcpp::Node
{
public:
  LidarRuleBasedClassifierNode()
  : Node("lidar_rule_based_classifier_node")
  {
    // Parameters
    this->declare_parameter("cluster_points_topic", "/cluster_points");
    this->declare_parameter("cluster_markers_topic", "/cluster_markers");
    this->declare_parameter("cluster_classifier_marker_topic", "/cluster_classifier_marker");
    this->declare_parameter("max_detection_distance", 50.0);
    this->declare_parameter("strict_distance_threshold", 30.0);
    this->declare_parameter("verbose", false);
    this->declare_parameter("detect_classes", std::vector<std::string>{"person", "car"});
    
    std::string cluster_points_topic = this->get_parameter("cluster_points_topic").as_string();
    std::string cluster_markers_topic = this->get_parameter("cluster_markers_topic").as_string();
    std::string classifier_marker_topic = this->get_parameter("cluster_classifier_marker_topic").as_string();
    max_detection_distance_ = this->get_parameter("max_detection_distance").as_double();
    strict_distance_threshold_ = this->get_parameter("strict_distance_threshold").as_double();
    verbose_ = this->get_parameter("verbose").as_bool();
    
    // Parse detect_classes parameter
    auto detect_classes_param = this->get_parameter("detect_classes").as_string_array();
    detect_classes_.clear();
    for (const auto& cls : detect_classes_param) {
      std::string cls_lower = cls;
      std::transform(cls_lower.begin(), cls_lower.end(), cls_lower.begin(), ::tolower);
      if (cls_lower == "person" || cls_lower == "car") {
        detect_classes_.push_back(cls_lower);
      }
    }
    if (detect_classes_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid detection classes specified, defaulting to all classes");
      detect_classes_ = {"person", "car"};
    }
    
    // Store flags for quick lookup
    detect_person_ = std::find(detect_classes_.begin(), detect_classes_.end(), "person") != detect_classes_.end();
    detect_car_ = std::find(detect_classes_.begin(), detect_classes_.end(), "car") != detect_classes_.end();
    
    // Subscribers
    cluster_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cluster_points_topic, 10,
      std::bind(&LidarRuleBasedClassifierNode::clusterPointsCallback, this, _1));
    
    cluster_markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      cluster_markers_topic, 10,
      std::bind(&LidarRuleBasedClassifierNode::clusterMarkersCallback, this, _1));
    
    // Publisher
    classifier_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      classifier_marker_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "LiDAR Rule-Based Classifier Node Started");
    RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s, %s", cluster_points_topic.c_str(), cluster_markers_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing to: %s", classifier_marker_topic.c_str());
    
    std::string classes_str;
    for (size_t i = 0; i < detect_classes_.size(); ++i) {
      if (i > 0) classes_str += ", ";
      classes_str += detect_classes_[i];
    }
    RCLCPP_INFO(this->get_logger(), "  Detection classes enabled: %s", classes_str.c_str());
    RCLCPP_INFO(this->get_logger(), "  Using rule-based classification only");
  }

private:
  void clusterPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg, *cloud);
      
      // Group points by cluster ID (stored in intensity field)
      cluster_dict_.clear();
      for (const auto& pt : cloud->points) {
        int cluster_id = static_cast<int>(pt.intensity);
        if (cluster_id > 0) {
          pcl::PointXYZ xyz_pt;
          xyz_pt.x = pt.x;
          xyz_pt.y = pt.y;
          xyz_pt.z = pt.z;
          cluster_dict_[cluster_id].push_back(xyz_pt);
        }
      }
      
      cluster_points_ = msg;
      
      // Process clusters when we have both points and markers
      if (cluster_markers_) {
        processAndClassify();
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing cluster points: %s", e.what());
    }
  }
  
  void clusterMarkersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    cluster_markers_ = msg;
    
    // Process clusters when we have both points and markers
    if (cluster_points_) {
      processAndClassify();
    }
  }
  
  ClusterFeatures extractFeatures(int cluster_id, const std::vector<pcl::PointXYZ>& points)
  {
    ClusterFeatures features;
    features.cluster_id = cluster_id;
    features.points = points;
    features.num_points = points.size();
    
    if (points.empty()) {
      return features;
    }
    
    // Compute bounding box - manually compute min/max from vector
    pcl::PointXYZ min_pt, max_pt;
    if (!points.empty()) {
      min_pt = points[0];
      max_pt = points[0];
      for (const auto& pt : points) {
        if (pt.x < min_pt.x) min_pt.x = pt.x;
        if (pt.y < min_pt.y) min_pt.y = pt.y;
        if (pt.z < min_pt.z) min_pt.z = pt.z;
        if (pt.x > max_pt.x) max_pt.x = pt.x;
        if (pt.y > max_pt.y) max_pt.y = pt.y;
        if (pt.z > max_pt.z) max_pt.z = pt.z;
      }
    }
    
    features.height = max_pt.z - min_pt.z;
    
    // Compute centroid manually
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    for (const auto& pt : points) {
      sum_x += pt.x;
      sum_y += pt.y;
      sum_z += pt.z;
    }
    size_t num_points = points.size();
    if (num_points > 0) {
      features.centroid = Eigen::Vector3d(sum_x / num_points, sum_y / num_points, sum_z / num_points);
    }
    features.distance_from_origin = features.centroid.norm();
    
    // Bounding box dimensions
    features.bounding_box.length = max_pt.x - min_pt.x;
    features.bounding_box.width = max_pt.y - min_pt.y;
    features.bounding_box.height = max_pt.z - min_pt.z;
    
    features.volume = features.bounding_box.length * features.bounding_box.width * features.bounding_box.height;
    features.aspect_ratio = (features.bounding_box.width > 0) ? 
                           (features.bounding_box.length / features.bounding_box.width) : 0.0;
    
    // Statistical features - reuse sum_z from centroid calculation
    features.mean_height = (num_points > 0) ? (sum_z / num_points) : 0.0;
    
    double sum_sq_diff = 0.0;
    for (const auto& pt : points) {
      double diff = pt.z - features.mean_height;
      sum_sq_diff += diff * diff;
    }
    features.std_height = std::sqrt(sum_sq_diff / points.size());
    
    features.point_density = (features.volume > 0) ? 
                            (features.num_points / features.volume) : 0.0;
    
    // PCA-based shape features using Eigen
    try {
      // Center the points
      Eigen::MatrixXd centered_points(points.size(), 3);
      for (size_t i = 0; i < points.size(); ++i) {
        centered_points(i, 0) = points[i].x - features.centroid[0];
        centered_points(i, 1) = points[i].y - features.centroid[1];
        centered_points(i, 2) = points[i].z - features.centroid[2];
      }
      
      // Compute covariance matrix
      Eigen::Matrix3d cov_matrix = (centered_points.transpose() * centered_points) / (points.size() - 1);
      
      // Compute eigenvalues
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov_matrix);
      if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("Eigenvalue computation failed");
      }
      
      Eigen::Vector3d evals = eigensolver.eigenvalues();
      // Sort eigenvalues (largest to smallest)
      std::vector<double> eval_vec = {evals[0], evals[1], evals[2]};
      std::sort(eval_vec.begin(), eval_vec.end(), std::greater<double>());
      
      double lambda1 = std::max(eval_vec[0], 0.0);
      double lambda2 = std::max(eval_vec[1], 0.0);
      double lambda3 = std::max(eval_vec[2], 0.0);
      
      // Avoid division by zero
      const double epsilon = 1e-10;
      lambda1 += epsilon;
      lambda2 += epsilon;
      lambda3 += epsilon;
      
      features.eigenvalues = Eigen::Vector3d(lambda1, lambda2, lambda3);
      features.eigenvalue_ratios = Eigen::Vector2d(lambda2 / lambda1, lambda3 / lambda2);
      
      // Shape features
      features.linearity = (lambda1 - lambda2) / lambda1;
      features.planarity = (lambda2 - lambda3) / lambda1;
      features.sphericity = lambda3 / lambda1;
      
    } catch (const std::exception& e) {
      if (verbose_) {
        RCLCPP_WARN(this->get_logger(), "Error computing PCA for cluster %d: %s", cluster_id, e.what());
      }
      // Set default values
      features.eigenvalues = Eigen::Vector3d(1.0, 1.0, 1.0);
      features.eigenvalue_ratios = Eigen::Vector2d(1.0, 1.0);
      features.linearity = 0.0;
      features.planarity = 0.0;
      features.sphericity = 1.0;
    }
    
    return features;
  }
  
  bool classifyPerson(const ClusterFeatures& features)
  {
    if (features.num_points == 0) {
      return false;
    }
    
    // Distance-based filtering
    if (features.distance_from_origin > max_detection_distance_) {
      return false;
    }
    
    bool is_far = features.distance_from_origin > strict_distance_threshold_;
    double distance_factor = is_far ? 0.9 : 1.0;
    
    // Rule 1: Height check
    bool height_valid = (1.3 <= features.height && features.height <= 2.2);
    
    // Rule 2: Centroid height
    double centroid_height = features.centroid[2];
    bool centroid_height_valid = (0.5 <= centroid_height && centroid_height <= 2.0);
    
    // Rule 3: Volume check
    bool volume_valid = (0.1 <= features.volume && features.volume <= 5.0);
    
    // Rule 4: Height is largest dimension
    std::vector<double> dims = {features.bounding_box.length, 
                                features.bounding_box.width, 
                                features.bounding_box.height};
    std::sort(dims.begin(), dims.end(), std::greater<double>());
    bool height_is_largest = (features.bounding_box.height == dims[0]);
    
    // Rule 5: Point count
    bool point_count_valid = (30 <= features.num_points && features.num_points <= 2000);
    
    // Rule 6: Point density
    bool density_valid = (100 <= features.point_density && features.point_density <= 50000);
    
    // Rule 7: Vertical orientation
    double width_length_avg = (features.bounding_box.width + features.bounding_box.length) / 2.0;
    double height_ratio = (width_length_avg > 0) ? 
                         (features.bounding_box.height / width_length_avg) : 0.0;
    bool vertical_orientation = (height_ratio >= 1.3);
    
    // Rule 8: Maximum width constraint
    double max_width = std::max(features.bounding_box.width, features.bounding_box.length);
    double max_width_threshold = 0.85 * distance_factor;
    bool max_width_valid = (max_width <= max_width_threshold);
    
    // Rule 9: Footprint area
    double footprint_area = features.bounding_box.width * features.bounding_box.length;
    double footprint_threshold = 0.6 * distance_factor;
    bool footprint_valid = (footprint_area <= footprint_threshold);
    
    // Rule 10: Shape check
    bool shape_valid = (features.planarity < 0.7 && features.sphericity > 0.2);
    
    // Rule 11: Width and length check
    double min_dim = std::min(features.bounding_box.width, features.bounding_box.length);
    double max_dim = std::max(features.bounding_box.width, features.bounding_box.length);
    double width_length_ratio = (min_dim > 0) ? (max_dim / min_dim) : 100.0;
    bool width_length_valid = (width_length_ratio <= 2.2);
    
    // CRITICAL RULES (must pass all)
    std::vector<bool> critical_rules = {
      height_valid,
      height_is_largest,
      vertical_orientation,
      max_width_valid,
      footprint_valid
    };
    bool all_critical_pass = std::all_of(critical_rules.begin(), critical_rules.end(), [](bool v) { return v; });
    
    // IMPORTANT RULES
    std::vector<bool> important_rules = {
      centroid_height_valid,
      volume_valid,
      point_count_valid,
      density_valid,
      shape_valid,
      width_length_valid
    };
    int important_rules_passed = std::count(important_rules.begin(), important_rules.end(), true);
    
    bool is_person = all_critical_pass && (important_rules_passed >= 4);
    
    if (verbose_ && is_person) {
      std::string distance_note = is_far ? 
        (" (far: " + std::to_string(features.distance_from_origin) + "m)") :
        (" (near: " + std::to_string(features.distance_from_origin) + "m)");
      RCLCPP_INFO(this->get_logger(),
        "Person detected! Cluster %d%s: height=%.2fm, width=%.2fm, length=%.2fm, "
        "footprint=%.3fm², volume=%.2fm³, height_ratio=%.2f, points=%zu, "
        "critical_rules=%zu/5, important_rules=%d/6",
        features.cluster_id, distance_note.c_str(), features.height,
        features.bounding_box.width, features.bounding_box.length, footprint_area,
        features.volume, height_ratio, features.num_points,
        std::count(critical_rules.begin(), critical_rules.end(), true),
        important_rules_passed);
    }
    
    return is_person;
  }
  
  bool classifyCar(const ClusterFeatures& features)
  {
    if (features.num_points == 0) {
      return false;
    }
    
    // Distance-based filtering
    if (features.distance_from_origin > max_detection_distance_) {
      return false;
    }
    
    bool is_far = features.distance_from_origin > strict_distance_threshold_;
    
    // Calculate key metrics upfront
    double footprint_area = features.bounding_box.width * features.bounding_box.length;
    double length_width_ratio = (features.bounding_box.width > 0) ? 
                               (features.bounding_box.length / features.bounding_box.width) : 0.0;
    double height_length_ratio = (features.bounding_box.length > 0) ? 
                                (features.bounding_box.height / features.bounding_box.length) : 0.0;
    double height_width_ratio = (features.bounding_box.width > 0) ? 
                               (features.bounding_box.height / features.bounding_box.width) : 100.0;
    double volume_footprint_ratio = (footprint_area > 0) ? 
                                   (features.volume / footprint_area) : 0.0;
    
    // Rule 1: Height check
    bool height_valid = (1.0 <= features.height && features.height <= 2.2);
    
    // Rule 2: Centroid height
    double centroid_height = features.centroid[2];
    bool centroid_height_valid = (0.2 <= centroid_height && centroid_height <= 1.2);
    
    // Rule 2b: Ground contact check
    double min_z = std::numeric_limits<double>::max();
    for (const auto& pt : features.points) {
      if (pt.z < min_z) min_z = pt.z;
    }
    bool ground_contact_valid = (min_z <= 0.3);
    
    // Rule 3: Volume check
    bool volume_valid = (2.0 <= features.volume && features.volume <= 55.0);
    
    // Rule 4: Length is largest dimension
    std::vector<double> dims = {features.bounding_box.length, 
                                features.bounding_box.width, 
                                features.bounding_box.height};
    std::sort(dims.begin(), dims.end(), std::greater<double>());
    bool length_is_largest = (features.bounding_box.length == dims[0]);
    
    // Rule 5: Minimum length
    bool min_length_valid = (features.bounding_box.length >= 2.8);
    
    // Rule 6: Maximum length
    bool max_length_valid = (features.bounding_box.length < 7.0);
    
    // Rule 7: Width check
    bool width_valid = (1.5 <= features.bounding_box.width && features.bounding_box.width <= 2.4);
    
    // Rule 8: Length to width ratio
    bool elongated_valid = (length_width_ratio >= 1.6);
    
    // Rule 9: Point count
    bool point_count_valid = (250 <= features.num_points && features.num_points <= 18000);
    
    // Rule 10: Point density
    bool density_valid = (40 <= features.point_density && features.point_density <= 25000);
    
    // Rule 11: Footprint area
    bool footprint_valid;
    if (is_far) {
      footprint_valid = (4.5 <= footprint_area && footprint_area <= 14.0);
    } else {
      footprint_valid = (3.5 <= footprint_area && footprint_area <= 16.0);
    }
    
    // Rule 12: Height to length ratio
    bool low_profile_valid = (height_length_ratio <= 0.65);
    
    // Rule 13: Shape check
    bool shape_valid = (features.planarity > 0.2);
    
    // Rule 14: Not too tall
    bool not_too_tall = (height_width_ratio <= 1.5);
    
    // Rule 15: Volume proportional to footprint
    bool volume_proportional = (0.7 <= volume_footprint_ratio && volume_footprint_ratio <= 2.5);
    
    // Rule 16: Maximum height constraint
    bool max_height_valid = (features.bounding_box.height <= 2.2);
    
    // Rule 17: Explicit truck rejection
    bool not_a_truck = (features.bounding_box.length < 7.0) && 
                      !(features.bounding_box.width >= 1.8 && footprint_area >= 12.0);
    
    // CRITICAL RULES
    std::vector<bool> critical_rules = {
      height_valid,
      length_is_largest,
      min_length_valid,
      elongated_valid,
      footprint_valid,
      width_valid,
      ground_contact_valid,
      not_a_truck
    };
    bool all_critical_pass = std::all_of(critical_rules.begin(), critical_rules.end(), [](bool v) { return v; });
    
    // IMPORTANT RULES
    std::vector<bool> important_rules = {
      centroid_height_valid,
      volume_valid,
      max_length_valid,
      point_count_valid,
      density_valid,
      low_profile_valid,
      shape_valid,
      not_too_tall,
      volume_proportional,
      max_height_valid
    };
    int important_rules_passed = std::count(important_rules.begin(), important_rules.end(), true);
    
    bool is_car = all_critical_pass && 
                 ((is_far && important_rules_passed >= 7) || (!is_far && important_rules_passed >= 6));
    
    if (verbose_ && is_car) {
      std::string distance_note = is_far ? 
        (" (far: " + std::to_string(features.distance_from_origin) + "m)") :
        (" (near: " + std::to_string(features.distance_from_origin) + "m)");
      RCLCPP_INFO(this->get_logger(),
        "Car detected! Cluster %d%s: height=%.2fm, width=%.2fm, length=%.2fm, "
        "footprint=%.3fm², volume=%.2fm³, centroid_z=%.2fm, bottom_z=%.2fm, "
        "L/W_ratio=%.2f, H/L_ratio=%.2f, H/W_ratio=%.2f, points=%zu, "
        "critical_rules=%zu/8, important_rules=%d/10",
        features.cluster_id, distance_note.c_str(), features.height,
        features.bounding_box.width, features.bounding_box.length, footprint_area,
        features.volume, centroid_height, min_z, length_width_ratio,
        height_length_ratio, height_width_ratio, features.num_points,
        std::count(critical_rules.begin(), critical_rules.end(), true),
        important_rules_passed);
    }
    
    return is_car;
  }
  
  std::vector<visualization_msgs::msg::Marker> createWireframeBboxMarker(
    const ClusterFeatures& features, const std::string& classification)
  {
    std::vector<visualization_msgs::msg::Marker> markers;
    
    // Get bounding box - manually compute min/max from vector
    pcl::PointXYZ min_pt, max_pt;
    if (!features.points.empty()) {
      min_pt = features.points[0];
      max_pt = features.points[0];
      for (const auto& pt : features.points) {
        if (pt.x < min_pt.x) min_pt.x = pt.x;
        if (pt.y < min_pt.y) min_pt.y = pt.y;
        if (pt.z < min_pt.z) min_pt.z = pt.z;
        if (pt.x > max_pt.x) max_pt.x = pt.x;
        if (pt.y > max_pt.y) max_pt.y = pt.y;
        if (pt.z > max_pt.z) max_pt.z = pt.z;
      }
    }
    
    // Define 8 vertices
    geometry_msgs::msg::Point p[8];
    p[0].x = min_pt.x; p[0].y = min_pt.y; p[0].z = min_pt.z;
    p[1].x = max_pt.x; p[1].y = min_pt.y; p[1].z = min_pt.z;
    p[2].x = max_pt.x; p[2].y = max_pt.y; p[2].z = min_pt.z;
    p[3].x = min_pt.x; p[3].y = max_pt.y; p[3].z = min_pt.z;
    p[4].x = min_pt.x; p[4].y = min_pt.y; p[4].z = max_pt.z;
    p[5].x = max_pt.x; p[5].y = min_pt.y; p[5].z = max_pt.z;
    p[6].x = max_pt.x; p[6].y = max_pt.y; p[6].z = max_pt.z;
    p[7].x = min_pt.x; p[7].y = max_pt.y; p[7].z = max_pt.z;
    
    // Create wireframe box marker
    visualization_msgs::msg::Marker bbox_marker;
    bbox_marker.header = cluster_points_->header;
    bbox_marker.ns = "cluster_bboxes";
    bbox_marker.id = features.cluster_id;
    bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    bbox_marker.action = visualization_msgs::msg::Marker::ADD;
    bbox_marker.pose.orientation.w = 1.0;
    bbox_marker.scale.x = 0.05;  // Line width
    
    // Set color based on classification
    if (classification == "person") {
      bbox_marker.color.r = 1.0;
      bbox_marker.color.g = 0.0;
      bbox_marker.color.b = 0.0;
      bbox_marker.color.a = 1.0;
    } else if (classification == "car") {
      bbox_marker.color.r = 0.0;
      bbox_marker.color.g = 1.0;
      bbox_marker.color.b = 0.0;
      bbox_marker.color.a = 1.0;
    } else {
      bbox_marker.color.r = 1.0;
      bbox_marker.color.g = 1.0;
      bbox_marker.color.b = 1.0;
      bbox_marker.color.a = 1.0;
    }
    
    // Add 12 edges of the box
    // Bottom face
    bbox_marker.points.push_back(p[0]); bbox_marker.points.push_back(p[1]);
    bbox_marker.points.push_back(p[1]); bbox_marker.points.push_back(p[2]);
    bbox_marker.points.push_back(p[2]); bbox_marker.points.push_back(p[3]);
    bbox_marker.points.push_back(p[3]); bbox_marker.points.push_back(p[0]);
    // Top face
    bbox_marker.points.push_back(p[4]); bbox_marker.points.push_back(p[5]);
    bbox_marker.points.push_back(p[5]); bbox_marker.points.push_back(p[6]);
    bbox_marker.points.push_back(p[6]); bbox_marker.points.push_back(p[7]);
    bbox_marker.points.push_back(p[7]); bbox_marker.points.push_back(p[4]);
    // Vertical edges
    bbox_marker.points.push_back(p[0]); bbox_marker.points.push_back(p[4]);
    bbox_marker.points.push_back(p[1]); bbox_marker.points.push_back(p[5]);
    bbox_marker.points.push_back(p[2]); bbox_marker.points.push_back(p[6]);
    bbox_marker.points.push_back(p[3]); bbox_marker.points.push_back(p[7]);
    
    markers.push_back(bbox_marker);
    
    // Create text label marker
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = cluster_points_->header;
    text_marker.ns = "cluster_labels";
    text_marker.id = features.cluster_id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = features.centroid[0];
    text_marker.pose.position.y = features.centroid[1];
    text_marker.pose.position.z = max_pt.z + 0.2;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.3;  // Text height
    
    // Convert to uppercase
    std::string text = classification;
    std::transform(text.begin(), text.end(), text.begin(), ::toupper);
    text_marker.text = text;
    
    // Match text color to bounding box color
    if (classification == "person") {
      text_marker.color.r = 1.0;
      text_marker.color.g = 0.0;
      text_marker.color.b = 0.0;
    } else if (classification == "car") {
      text_marker.color.r = 0.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0;
    } else {
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
    }
    text_marker.color.a = 1.0;
    
    markers.push_back(text_marker);
    
    return markers;
  }
  
  void processAndClassify()
  {
    if (!cluster_points_ || !cluster_markers_) {
      return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    std::map<int, std::string> current_classified_clusters;
    int person_count = 0;
    int car_count = 0;
    
    // Process each cluster
    for (const auto& [cluster_id, points] : cluster_dict_) {
      // Extract features
      ClusterFeatures features = extractFeatures(cluster_id, points);
      if (features.num_points == 0) {
        continue;
      }
      
      // Rule-based classification only
      std::string classification;
      bool is_person = false;
      bool is_car = false;
      
      // Check person if enabled
      if (detect_person_) {
        is_person = classifyPerson(features);
        if (is_person) {
          classification = "person";
          person_count++;
        }
      }
      
      // Check car if not a person and car detection is enabled
      if (!is_person && detect_car_) {
        is_car = classifyCar(features);
        if (is_car) {
          classification = "car";
          car_count++;
        }
      }
      
      if (!classification.empty()) {
        current_classified_clusters[cluster_id] = classification;
        
        // Create wireframe bounding box and text markers
        auto markers = createWireframeBboxMarker(features, classification);
        marker_array.markers.insert(marker_array.markers.end(), markers.begin(), markers.end());
      }
    }
    
    // Track current maximum cluster ID
    int current_max_id = 0;
    for (const auto& [id, _] : cluster_dict_) {
      current_max_id = std::max(current_max_id, id);
    }
    max_cluster_id_seen_ = std::max(max_cluster_id_seen_, current_max_id);
    
    // Delete markers for clusters that no longer exist
    for (const auto& [old_id, _] : classified_clusters_) {
      if (current_classified_clusters.find(old_id) == current_classified_clusters.end()) {
        // Delete bounding box marker
        visualization_msgs::msg::Marker bbox_delete;
        bbox_delete.header = cluster_points_->header;
        bbox_delete.ns = "cluster_bboxes";
        bbox_delete.id = old_id;
        bbox_delete.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(bbox_delete);
        
        // Delete text label marker
        visualization_msgs::msg::Marker text_delete;
        text_delete.header = cluster_points_->header;
        text_delete.ns = "cluster_labels";
        text_delete.id = old_id;
        text_delete.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(text_delete);
      }
    }
    
    // Delete markers for IDs beyond current max
    for (int i = current_max_id + 1; i <= max_cluster_id_seen_; ++i) {
      if (current_classified_clusters.find(i) == current_classified_clusters.end() &&
          classified_clusters_.find(i) == classified_clusters_.end()) {
        visualization_msgs::msg::Marker bbox_delete;
        bbox_delete.header = cluster_points_->header;
        bbox_delete.ns = "cluster_bboxes";
        bbox_delete.id = i;
        bbox_delete.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(bbox_delete);
        
        visualization_msgs::msg::Marker text_delete;
        text_delete.header = cluster_points_->header;
        text_delete.ns = "cluster_labels";
        text_delete.id = i;
        text_delete.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(text_delete);
      }
    }
    
    // Update tracked classifications
    classified_clusters_ = current_classified_clusters;
    
    // Publish markers
    if (!marker_array.markers.empty()) {
      classifier_marker_pub_->publish(marker_array);
    }
    
    if (verbose_) {
      if (person_count > 0) {
        RCLCPP_INFO(this->get_logger(), "Detected %d person(s)", person_count);
      }
      if (car_count > 0) {
        RCLCPP_INFO(this->get_logger(), "Detected %d car(s)", car_count);
      }
    }
  }
  
  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_points_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_markers_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr classifier_marker_pub_;
  
  // Data storage
  sensor_msgs::msg::PointCloud2::SharedPtr cluster_points_;
  visualization_msgs::msg::MarkerArray::SharedPtr cluster_markers_;
  std::map<int, std::vector<pcl::PointXYZ>> cluster_dict_;
  std::map<int, std::string> classified_clusters_;
  int max_cluster_id_seen_ = 0;
  
  // Parameters
  std::vector<std::string> detect_classes_;
  bool detect_person_ = false;
  bool detect_car_ = false;
  double max_detection_distance_ = 50.0;
  double strict_distance_threshold_ = 30.0;
  bool verbose_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRuleBasedClassifierNode>());
  rclcpp::shutdown();
  return 0;
}

