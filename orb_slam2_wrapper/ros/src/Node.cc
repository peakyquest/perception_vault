/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Node.hpp"

#include <iostream>
#include <cstdlib>
#include <filesystem>

Node::Node(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  image_transport_(nullptr),
  map_points_publisher_(nullptr),
  pose_publisher_(nullptr),
  camera_info_sub_(nullptr),
  service_server_(nullptr),
  node_name_(node_name),
  initialized_(false),
  min_observations_per_point_(2)
{
  declare_parameter("publish_pointcloud", rclcpp::ParameterValue(true));
  declare_parameter("publish_pose", rclcpp::ParameterValue(true));
  declare_parameter("publish_tf", rclcpp::ParameterValue(true));
  declare_parameter("publish_trajectory", rclcpp::ParameterValue(false));
  declare_parameter("pointcloud_frame_id", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("camera_frame_id", rclcpp::ParameterValue(std::string("camera_link")));
  declare_parameter("map_file", rclcpp::ParameterValue(std::string("map.bin")));
  declare_parameter("voc_file", rclcpp::ParameterValue(std::string("file_not_set")));
  declare_parameter("load_map", rclcpp::ParameterValue(false));

  // ORB SLAM configuration parameters
  declare_parameter("camera_fps", rclcpp::ParameterValue(30));
  declare_parameter("camera_rgb_encoding", rclcpp::ParameterValue(true));
  declare_parameter("ORBextractor/nFeatures", rclcpp::ParameterValue(1200));
  declare_parameter("ORBextractor/scaleFactor", rclcpp::ParameterValue(1.2f));
  declare_parameter("ORBextractor/nLevels", rclcpp::ParameterValue(8));
  declare_parameter("ORBextractor/iniThFAST", rclcpp::ParameterValue(20));
  declare_parameter("ORBextractor/minThFAST", rclcpp::ParameterValue(7));
  declare_parameter("ThDepth", rclcpp::ParameterValue(35.0f));
  declare_parameter("depth_map_factor", rclcpp::ParameterValue(1.0f));
  declare_parameter("camera_baseline", rclcpp::ParameterValue(0.0f));
}

void Node::init(const ORB_SLAM2::System::eSensor & sensor)
{
  get_parameter("publish_pointcloud", publish_pointcloud_param_);
  get_parameter("publish_pose", publish_pose_param_);
  get_parameter("publish_tf", publish_tf_param_);
  get_parameter("publish_trajectory", publish_trajectory_param_);
  get_parameter("pointcloud_frame_id", map_frame_id_param_);
  get_parameter("camera_frame_id", camera_frame_id_param_);
  get_parameter("map_file", map_file_name_param_);
  get_parameter("voc_file", voc_file_name_param_);
  get_parameter("load_map", load_map_param_);

  sensor_ = sensor;

  image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  rendered_image_publisher_ = image_transport_->advertise(node_name_ + "/debug_image", 1);
  if (publish_pointcloud_param_) {
    map_points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        node_name_ + "/map_points", 1);
  }

  // Enable publishing camera's pose as PoseStamped message
  if (publish_pose_param_) {
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(node_name_ + "/pose", 1);
  }

  // Enable publishing camera's trajectory as Path message
  if (publish_trajectory_param_) {
    trajectory_publisher_ = create_publisher<nav_msgs::msg::Path>(node_name_ + "/trajectory", 1);
    trajectory_path_.header.frame_id = map_frame_id_param_;
  }

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 1, std::bind(&Node::cameraInfoCallback, this, std::placeholders::_1));

  service_server_ = create_service<orb_slam2_wrapper::srv::SaveMap>(
      node_name_ + "/save_map",
      std::bind(
          &Node::SaveMapSrv, this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

Node::~Node()
{
  // Stop all threads
  orb_slam_->Shutdown();

  // Save camera trajectory
  orb_slam_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  delete orb_slam_;
}

void Node::Update()
{
  if (!initialized_) {
    RCLCPP_WARN(get_logger(), "Camera info not received, node has not been initialized!");
    return;
  }

  cv::Mat position = orb_slam_->GetCurrentPosition();

  if (!position.empty()) {
    PublishPositionAsTransform(position);
    if (publish_pose_param_) {
      PublishPositionAsPoseStamped(position);
    }
    if (publish_trajectory_param_) {
      PublishTrajectory(position);
    }
  }

  PublishRenderedImage(orb_slam_->DrawCurrentFrame());
  if (publish_pointcloud_param_) {
    PublishMapPoints(orb_slam_->GetAllMapPoints());
  }
}

void Node::PublishMapPoints(std::vector<ORB_SLAM2::MapPoint *> map_points)
{
  sensor_msgs::msg::PointCloud2 cloud = MapPointsToPointCloud(map_points);
  map_points_publisher_->publish(cloud);
}

void Node::PublishPositionAsTransform(cv::Mat position)
{
  if (publish_tf_param_) {
    tf2::Transform transform = TransformFromMat(position);
    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = map_frame_id_param_;
    tmp_tf_stamped.header.stamp = current_frame_time_;
    tmp_tf_stamped.child_frame_id = camera_frame_id_param_;
    tf2::impl::Converter<false, true>::convert(transform, tmp_tf_stamped.transform);
    tf_broadcaster_->sendTransform(tmp_tf_stamped);
  }
}

void Node::PublishPositionAsPoseStamped(cv::Mat position)
{
  tf2::Transform grasp_tf = TransformFromMat(position);
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = current_frame_time_;
  pose_msg.header.frame_id = map_frame_id_param_;
  tf2::toMsg(grasp_tf, pose_msg.pose);
  pose_publisher_->publish(pose_msg);
}

void Node::PublishTrajectory(cv::Mat position)
{
  tf2::Transform grasp_tf = TransformFromMat(position);
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = current_frame_time_;
  pose_msg.header.frame_id = map_frame_id_param_;
  tf2::toMsg(grasp_tf, pose_msg.pose);
  
  // Add the current pose to the trajectory
  trajectory_path_.poses.push_back(pose_msg);
  
  // Update the header timestamp
  trajectory_path_.header.stamp = current_frame_time_;
  trajectory_path_.header.frame_id = map_frame_id_param_;
  
  // Publish the trajectory
  trajectory_publisher_->publish(trajectory_path_);
}

void Node::PublishRenderedImage(cv::Mat image)
{
  std_msgs::msg::Header header;
  header.stamp = current_frame_time_;
  header.frame_id = map_frame_id_param_;
  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg =
    cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  rendered_image_publisher_.publish(rendered_image_msg);
}


tf2::Transform Node::TransformFromMat(cv::Mat position_mat)
{
  cv::Mat rotation(3, 3, CV_32F);
  cv::Mat translation(3, 1, CV_32F);

  rotation = position_mat.rowRange(0, 3).colRange(0, 3);
  translation = position_mat.rowRange(0, 3).col(3);


  tf2::Matrix3x3 tf_camera_rotation(
    rotation.at<float>(0, 0), rotation.at<float>(0, 1), rotation.at<float>(0, 2),
    rotation.at<float>(1, 0), rotation.at<float>(1, 1), rotation.at<float>(1, 2),
    rotation.at<float>(2, 0), rotation.at<float>(2, 1), rotation.at<float>(2, 2));

  tf2::Vector3 tf_camera_translation(
    translation.at<float>(0), translation.at<float>(1), translation.at<float>(2));

  // Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0);

  // Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

  // Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

  // Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros * tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros * tf_camera_translation;

  return tf2::Transform(tf_camera_rotation, tf_camera_translation);
}


sensor_msgs::msg::PointCloud2 Node::MapPointsToPointCloud(
  std::vector<ORB_SLAM2::MapPoint *> map_points)
{
  if (map_points.empty()) {
    RCLCPP_WARN(get_logger(), "Map point vector is empty!");
  }

  sensor_msgs::msg::PointCloud2 cloud;

  const int num_channels = 3;  // x y z

  cloud.header.stamp = current_frame_time_;
  cloud.header.frame_id = map_frame_id_param_;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = {"x", "y", "z"};
  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char * cloud_data_ptr = &(cloud.data[0]);

  float * data_array = new float[num_channels];
  for (unsigned int i = 0; i < cloud.width; i++) {
    if (map_points.at(i)->nObs >= min_observations_per_point_) {
      // x. Do the transformation by just reading at the position of z instead of x
      data_array[0] = map_points.at(i)->GetWorldPos().at<float>(2);
      // y. Do the transformation by just reading at the position of x instead of y
      data_array[1] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(0);
      // z. Do the transformation by just reading at the position of y instead of z
      data_array[2] = -1.0 * map_points.at(i)->GetWorldPos().at<float>(1);

      // TODO(tbd): dont hack the transformation but have a central conversion
      //  function for MapPointsToPointCloud and TransformFromMat

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
  }

  return cloud;
}

bool Node::SaveMapAsPCD(const std::string & file_path)
{
  // Retrieve current map points from ORB-SLAM2
  std::vector<ORB_SLAM2::MapPoint *> map_points = orb_slam_->GetAllMapPoints();

  if (map_points.empty()) {
    RCLCPP_WARN(get_logger(), "SaveMapAsPCD: map point vector is empty, nothing to save.");
    return false;
  }

  // Count valid points (respecting the same observation threshold as publishing)
  std::size_t valid_points = 0;
  for (auto * mp : map_points) {
    if (mp && mp->nObs >= min_observations_per_point_) {
      ++valid_points;
    }
  }

  if (valid_points == 0) {
    RCLCPP_WARN(get_logger(), "SaveMapAsPCD: no map points passed the observation threshold.");
    return false;
  }

  // Ensure the target directory exists
  try {
    std::filesystem::path p(file_path);
    if (p.has_parent_path()) {
      std::filesystem::create_directories(p.parent_path());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "SaveMapAsPCD: failed to create directories for %s: %s",
      file_path.c_str(), e.what());
    return false;
  }

  std::ofstream ofs(file_path);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(get_logger(), "SaveMapAsPCD: could not open file %s for writing.", file_path.c_str());
    return false;
  }

  // Write ASCII PCD header
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n";
  ofs << "VERSION 0.7\n";
  ofs << "FIELDS x y z\n";
  ofs << "SIZE 4 4 4\n";
  ofs << "TYPE F F F\n";
  ofs << "COUNT 1 1 1\n";
  ofs << "WIDTH " << valid_points << "\n";
  ofs << "HEIGHT 1\n";
  ofs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  ofs << "POINTS " << valid_points << "\n";
  ofs << "DATA ascii\n";

  // Write point data using the same coordinate convention as MapPointsToPointCloud
  for (auto * mp : map_points) {
    if (!mp || mp->nObs < min_observations_per_point_) {
      continue;
    }

    const cv::Mat & pos = mp->GetWorldPos();
    const float x = pos.at<float>(2);          // ORB z -> ROS x
    const float y = -1.0f * pos.at<float>(0);  // ORB x -> ROS y
    const float z = -1.0f * pos.at<float>(1);  // ORB y -> ROS z

    ofs << x << " " << y << " " << z << "\n";
  }

  ofs.close();

  RCLCPP_INFO(get_logger(), "Map was additionally saved as PCD to %s", file_path.c_str());
  return true;
}

void Node::SaveMapSrv(
  const shared_ptr<rmw_request_id_t>,
  const shared_ptr<orb_slam2_wrapper::srv::SaveMap::Request> request,
  const shared_ptr<orb_slam2_wrapper::srv::SaveMap::Response> response)
{
  response->success = orb_slam_->SaveMap(request->name);

  if (response->success) {
    RCLCPP_INFO(get_logger(), "Map was saved as %s", request->name.c_str());

    // Derive a PCD file path in ROS_HOME (or ~/.ros as fallback), mirroring where the
    // binary map is stored according to the README.
    std::filesystem::path base_path(request->name);
    // If the user provided an extension such as ".bin", drop it and use only the stem
    // for the PCD file name, so "map.bin" -> "map.pcd".
    std::string stem = base_path.stem().string();
    if (stem.empty()) {
      stem = request->name;
    }
    std::string pcd_file_name = stem + ".pcd";

    const char * ros_home_env = std::getenv("ROS_HOME");
    std::string ros_home;
    if (ros_home_env && std::strlen(ros_home_env) > 0) {
      ros_home = ros_home_env;
    } else {
      const char * home_env = std::getenv("HOME");
      if (home_env && std::strlen(home_env) > 0) {
        ros_home = std::string(home_env) + "/.ros";
      } else {
        ros_home = ".";
      }
    }

    std::filesystem::path pcd_path = std::filesystem::path(ros_home) / pcd_file_name;
    if (!SaveMapAsPCD(pcd_path.string())) {
      RCLCPP_WARN(get_logger(),
        "Map binary was saved but saving PCD to %s failed.", pcd_path.string().c_str());
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Map could not be saved.");
  }
}

void Node::cameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (!initialized_) {
    RCLCPP_INFO(get_logger(), "Camera info received.");
    LoadOrbParameters(msg);
  }
}

void Node::LoadOrbParameters(sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  // Create a parameters object to pass to the Tracking system
  ORB_SLAM2::ORBParameters parameters{};

  get_parameter("camera_fps", parameters.maxFrames);
  get_parameter("camera_rgb_encoding", parameters.RGB);
  get_parameter("ORBextractor/nFeatures", parameters.nFeatures);
  get_parameter("ORBextractor/scaleFactor", parameters.scaleFactor);
  get_parameter("ORBextractor/nLevels", parameters.nLevels);
  get_parameter("ORBextractor/iniThFAST", parameters.iniThFAST);
  get_parameter("ORBextractor/minThFAST", parameters.minThFAST);

  if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD) {
    get_parameter("ThDepth", parameters.thDepth);
    get_parameter("depth_map_factor", parameters.depthMapFactor);
  }

  parameters.fx = camera_info->k[0];
  parameters.fy = camera_info->k[4];
  parameters.cx = camera_info->k[2];
  parameters.cy = camera_info->k[5];
  // Guard against cameras that provide fewer than 5 distortion coefficients
  const auto & d = camera_info->d;
  parameters.k1 = d.size() > 0 ? d[0] : 0.0;
  parameters.k2 = d.size() > 1 ? d[1] : 0.0;
  parameters.p1 = d.size() > 2 ? d[2] : 0.0;
  parameters.p2 = d.size() > 3 ? d[3] : 0.0;
  parameters.k3 = d.size() > 4 ? d[4] : 0.0;

  if (sensor_ == ORB_SLAM2::System::STEREO || sensor_ == ORB_SLAM2::System::RGBD) {
    if (!get_parameter("camera_baseline", parameters.baseline)) {
      parameters.baseline = camera_info->p[3];
    }
  }

  orb_slam_ = new ORB_SLAM2::System(
      voc_file_name_param_,
      sensor_,
      parameters,
      map_file_name_param_,
      load_map_param_);

  initialized_ = true;
}
