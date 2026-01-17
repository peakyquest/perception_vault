#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#ifdef ROS_DISTRO_JAZZY
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#ifdef ROS_DISTRO_JAZZY
#include "image_transport/image_transport.hpp"
#else
#include "image_transport/image_transport.h"
#endif
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class ImageRectifier : public rclcpp::Node
{
public:
  ImageRectifier()
  : Node("image_rectifier")
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("input_camera_info_topic", "/camera/camera_info");
    this->declare_parameter<std::string>("output_image_topic", "/camera/image_rect");
    this->declare_parameter<int>("queue_size", 10);
  }

  void initialize()
  {
    // Get parameters
    std::string input_image_topic = this->get_parameter("input_image_topic").as_string();
    std::string input_camera_info_topic = this->get_parameter("input_camera_info_topic").as_string();
    std::string output_image_topic = this->get_parameter("output_image_topic").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();

    // Create subscriber for camera info
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      input_camera_info_topic,
      queue_size,
      std::bind(&ImageRectifier::cameraInfoCallback, this, std::placeholders::_1)
    );

    // Initialize image transport (now safe to use shared_from_this() since node is in shared_ptr)
    image_transport_it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

    // Create subscriber for input images
    image_sub_ = image_transport_it_->subscribe(
      input_image_topic,
      queue_size,
      std::bind(&ImageRectifier::imageCallback, this, std::placeholders::_1)
    );

    // Create publisher for rectified images
    image_pub_ = image_transport_it_->advertise(output_image_topic, queue_size);

    RCLCPP_INFO(this->get_logger(), "Image rectifier node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: %s", input_image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to camera info topic: %s", input_camera_info_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing rectified images to: %s", output_image_topic.c_str());
  }

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (maps_computed_) {
      return;  // Already computed, no need to recompute
    }

    // Extract camera matrix
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = msg->k[0];  // fx
    camera_matrix.at<double>(0, 1) = msg->k[1];  // skew
    camera_matrix.at<double>(0, 2) = msg->k[2];  // cx
    camera_matrix.at<double>(1, 0) = msg->k[3];  // 0
    camera_matrix.at<double>(1, 1) = msg->k[4];  // fy
    camera_matrix.at<double>(1, 2) = msg->k[5];  // cy
    camera_matrix.at<double>(2, 0) = msg->k[6];  // 0
    camera_matrix.at<double>(2, 1) = msg->k[7];  // 0
    camera_matrix.at<double>(2, 2) = msg->k[8];  // 1

    // Extract distortion coefficients
    cv::Mat dist_coeffs = cv::Mat(msg->d.size(), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      dist_coeffs.at<double>(i) = msg->d[i];
    }

    // Use provided camera matrix or compute new optimal one
    cv::Size image_size(msg->width, msg->height);
    cv::Mat new_camera_matrix;

    if (msg->roi.width == 0 && msg->roi.height == 0) {
      // No ROI specified, use original camera matrix
      new_camera_matrix = camera_matrix;
    } else {
      // Compute optimal camera matrix
      new_camera_matrix = cv::getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, image_size, 0.0, image_size
      );
    }

    // Compute rectification maps
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);  // Identity rotation matrix for simple undistortion

    cv::initUndistortRectifyMap(
      camera_matrix,
      dist_coeffs,
      R,
      new_camera_matrix,
      image_size,
      CV_16SC2,
      map1_,
      map2_
    );

    camera_matrix_ = new_camera_matrix;
    maps_computed_ = true;

    RCLCPP_INFO(this->get_logger(), "Camera calibration parameters loaded and rectification maps computed");
    RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", image_size.width, image_size.height);
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!maps_computed_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Camera info not yet received, cannot rectify images");
      return;
    }

    try {
      // Convert ROS image message to OpenCV format
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);

      // Rectify the image
      cv::Mat rectified_image;
      cv::remap(cv_ptr->image, rectified_image, map1_, map2_, cv::INTER_LINEAR);

      // Convert back to ROS message
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = rectified_image;

      // Publish rectified image
      image_pub_.publish(out_msg.toImageMsg());

    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  cv::Mat map1_, map2_;
  cv::Mat camera_matrix_;
  bool maps_computed_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageRectifier>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

